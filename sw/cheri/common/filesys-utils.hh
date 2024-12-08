/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cheri.hh>

#include <assert.h>

#include "console.hh"
#include "sdcard-utils.hh"

/**
 * Very simple layer for read access to the files within the root directory
 * of a FAT32 partition on an SD card.
 *
 * If a more sophisticated, feature-rich filing system layer including, e.g.
 * support for writing data, is required, there are a number of open source
 * implementations of FAT32 support available.
 *
 * The code will locate the first FAT32 paritition, and only Master Boot Record (MBR)
 * partitioning is supported, which is how blanks microSD cards are shipped by
 * manufacturers, so avoid the use of GPT if reformatting.
 *
 * https://en.wikipedia.org/wiki/File_Allocation_Table#FAT32
 * https://en.wikipedia.org/wiki/Design_of_the_FAT_file_system
 */

class fileSysUtils {
 private:
  // Access to debug/diagnostic logging.
  Log *log;
  // SD card access.
  SdCard *sd;

  // Some SD cards support only a 512-byte block size, and SPI mode transfers are
  // always in terms of that anyway.
  static constexpr unsigned kBytesPerBlockShift = 9u;
  static constexpr unsigned kBlockLen           = (1u << kBytesPerBlockShift);

  // Properties of the FAT32 partition.
  bool partValid;
  // The logical volume consists of sectors, which are not necessarily the same size as
  // the blocks used at (SD card) driver level.
  uint8_t bytesPerSectorShift;
  uint8_t secsPerClusterShift;
  uint8_t blksPerClusterShift;
  // First block of the FAT, relative to the medium start.
  uint32_t fatStart;
  // First block of the cluster heap.
  uint32_t clusterHeapStart;
  // First block of the root directory.
  uint32_t rootStart;
  // First cluster holding the root directory.
  uint32_t rootCluster;
  // Cluster size in bytes.
  uint32_t clusterBytes;
  // Mask used to extract the byte offset within the current cluster
  //   (= cluster size in bytes - 1).
  uint32_t clusterMask;

  // Single block buffer for use when reading partitions and FAT contents; this is a 512-byte
  // block as required by SPI mode SD card access, which is conveniently enough to hold the
  // longest LFN (255 UCS-2 characters, plus terminator) after initialisation.
  union {
    uint8_t dataBuffer[kBlockLen];
    uint16_t nameBuffer[0x100u];
  } buf;

  // Number of entries in the block cache.
  static constexpr unsigned kCacheEntries = 8u;
  // Denotes an unused entry in the block cache.
  static constexpr uint32_t kInvalidBlock = ~(uint32_t)0u;
  // Block cache for next eviction.
  unsigned blockCacheNext;
  // Each block within the cache.
  struct {
    // Block number of the data occuping this cache entry (or kInvalidBlock).
    uint32_t block;
    // Data for this block.
    uint8_t buf[kBlockLen];
  } blockCache[kCacheEntries];

  // Object state flags.
  enum { Flag_Valid = 1U << 31 };

  // State information on an object being accessed; this may be either a file or a directory.
  struct objState {
    // Flags specifying validity/properties of this object.
    uint32_t flags;
    // Current offset (bytes) within the object.
    uint32_t offset;
    // Object length in bytes.
    uint32_t length;
    // Cluster number of the cluster holding the data at the current offset.
    uint32_t currCluster;
    // Cluster number of the first cluster holding the data for this object.
    uint32_t firstCluster;
  };

  // Set of open files.
  static constexpr unsigned kMaxFiles = 4u;
  objState files[kMaxFiles];

  // Set of open directories.
  static constexpr unsigned kMaxDirs = 2u;
  objState dirs[kMaxDirs];

  // Copy a sequence of bytes; destination and source must _not_ overlap.
  static void copy_bytes(uint8_t *dst, const uint8_t *src, size_t len) {
    const uint8_t *esrc = src + len;
    // Check there is no overlap between source and destination buffers;
    // this expression avoids issues with address addition wrapping.
    assert(dst < src || dst - src >= len);
    assert(src < dst || src - dst >= len);
    while (src < esrc) {
      *dst++ = *src++;
    }
  }

  // Ensure that the specified block is available in memory for access.
  int block_ensure(uint32_t block) {
    // Check whether this block is already available.
    int idx = 0;
    while (idx < kCacheEntries) {
      if (block == blockCache[idx].block) {
        return idx;
      }
      idx++;
    }
    idx = blockCacheNext;
    if (log) {
      log->println(" (reading blk {:#x})", block);
    }
    if (sd->read_blocks(block, blockCache[idx].buf, 1u)) {
      blockCache[idx].block = block;
      // Round-robin replacement of cached blocks.
      if (++blockCacheNext >= kCacheEntries) {
        blockCacheNext = 0u;
      }
      return idx;
    }
    return -1;
  }

  // Is the specified cluster number an End of Chain marker?
  // (a number of different values are used as EOC markers.)
  inline bool end_of_chain(uint32_t cluster) { return (cluster <= 1u) || (cluster >= 0x0ffffff8u); }

  // Read the next cluster in the cluster chain of an object.
  bool cluster_next(uint32_t &nextCluster, uint32_t cluster) {
    // Byte offset of the corresponding entry within the FAT.
    uint32_t byteOffset = cluster << 2;
    // Determine the block number of the part of the FAT that describes this cluster.
    uint32_t block = fatStart + (byteOffset >> kBytesPerBlockShift);
    int idx        = block_ensure(block);
    if (idx < 0) {
      // Failed to read the block from the medium.
      return false;
    }
    nextCluster = read32le(&blockCache[idx].buf[byteOffset & (kBlockLen - 1u)]);
    // The upper nibble of the cluster must be ignored; reserved for future use.
    nextCluster &= ~0xf0000000u;
    return true;
  }

  // Seek to the given offset within an object (file/directory).
  bool object_seek(objState &obj, uint32_t offset) {
    // First validate the requested offset.
    if (offset > obj.length) {
      return false;
    }
    // Start either from the current file offset (trusted) or the beginning of the file.
    uint32_t currCluster = obj.currCluster;
    uint32_t currOffset  = obj.offset & ~clusterMask;
    if (offset < currOffset) {
      currCluster = obj.firstCluster;
      currOffset  = 0u;
    }
    // Scan forwards through the cluster chain until we find the correct cluster.
    while (offset - currOffset >= clusterBytes) {
      uint32_t nextCluster;
      if (!cluster_next(nextCluster, currCluster)) {
        // Leave the current position unchanged.
        return false;
      }
      currCluster = nextCluster;
      currOffset += clusterBytes;
    }
    // Atomically update the current position with a consistent cluster number and offset.
    obj.currCluster = currCluster;
    obj.offset      = offset;
    return true;
  }

  // Read a contiguous sequence of bytes from an object (file/directory).
  size_t object_read(objState &obj, uint8_t *buf, size_t len) {
    if (log) {
      log->println("reading {:#x} byte(s) at offset {:#x}", len, obj.offset);
    }

    size_t bytesRead = 0u;
    while (len > 0u && obj.offset < obj.length) {
      uint32_t currBlock = block_number(obj.currCluster, obj.offset & clusterMask);

      // Ensure that the block containing the current offset is available for use, if it
      // can be read from the medium.
      int idx = block_ensure(currBlock);
      if (idx < 0) {
        return bytesRead;
      }
      // Locate this block within the block cache; its availability is guaranteed at this point.
      const uint8_t *dataBuf = blockCache[idx].buf;

      // How much data do we have available at the current offset?
      size_t blockOffset    = obj.offset & (kBlockLen - 1u);
      size_t blockBytesLeft = kBlockLen - blockOffset;
      size_t objBytesLeft   = obj.length - obj.offset;
      size_t bytesAvail     = (objBytesLeft > blockBytesLeft) ? blockBytesLeft : objBytesLeft;
      // Limit this request to the bytes immediately available.
      size_t chunk_len = (len > bytesAvail) ? bytesAvail : len;

      // Have we reached the end of this cluster but not the end of the object data?
      uint32_t next_offset = obj.offset + chunk_len;
      if (!(next_offset & clusterMask) && obj.length > next_offset) {
        uint32_t nextCluster;
        if (!cluster_next(nextCluster, obj.currCluster)) {
          // Note: we're leaving the object state consistent here, despite the read failure.
          return bytesRead;
        }
        // Store the updated cluster number for the new offset.
        obj.currCluster = nextCluster;
      }
      // Advance the current offset, now that we know that the new offset is consistent wtih the
      // cluster number.
      obj.offset += chunk_len;

      // We have no memcpy implementation presently.
      copy_bytes(buf, &dataBuf[blockOffset], chunk_len);
      buf += chunk_len;
      len -= chunk_len;
      bytesRead += chunk_len;
    }
    return bytesRead;
  }

  // Unfortunately FAT stores the literal values for bytes/sector and sectors/cluster but only
  // powers of two are permitted.
  static inline uint8_t floor_log2(uint16_t n) {
    uint8_t shift = 0u;
    while (n > 1u) {
      n >>= 1;
      shift++;
    }
    return shift;
  }

 public:
  // Opaque handle to an open file.
  typedef uint8_t fileHandle;
  // Invalid file handle, returned by a failed `file_open` call.
  static constexpr uint8_t kInvalidFileHandle = 0xffu;

  // Opaque handle to an open directory.
  typedef uint8_t dirHandle;
  // Invalid directory handle, returned by a failed 'dir_open' call.
  static constexpr uint8_t kInvalidDirHandle = 0xffu;

  // Flags specifying the type of directory access required.
  enum dirFlags {
    DirFlag_Raw            = 1u,
    DirFlag_IncludeDeleted = 2u,
    DirFlag_IncludeHidden  = 4u,

    DirFlags_Default = 0u,
  };

  // Directory entry type flags; this just makes the most common types of entries more accessible.
  enum dirEntryFlags {
    DirEntryFlag_Deleted      = 1u,
    DirEntryFlag_Hidden       = 2u,
    DirEntryFlag_VolumeLabel  = 4u,
    DirEntryFlag_Subdirectory = 8u,
    DirEntryFlag_HasLongName  = 0x10u
  };

  // Description of an entry within a directory object.
  struct dirEntry {
    dirEntryFlags flags;
    uint8_t entryType;
    // Short name of this object (8.3 format)
    // Note: these fields are padded with spaces (0x20) and there is no NUL terminator.
    uint8_t shortName[8];
    uint8_t shortExt[8];
    // See the FAT file system design for the interpretation of the following fields.
    uint8_t attribs;
    uint8_t userAttribs;
    uint8_t createdFine;
    uint16_t createdTime;
    uint16_t createdDate;
    uint16_t modifiedTime;
    uint16_t modifiedDate;
    uint16_t accessDate;
    // Cluster number of the first cluster holding this object's data.
    uint32_t firstCluster;
    // Length of the object in bytes.
    uint32_t dataLength;
  };

  fileSysUtils(Log *log_ = nullptr) : log(log_), sd(nullptr) {
    // Initialise all state information; no partition details, empty block cache,
    // no file/dir handles.
    fin();
  }

  // Test for the presence of a FAT32 partition, read the partition properties
  // and then locate the cluster heap and root directory.
  bool init(SdCard *sd_) {
    /// Retain access to the SD card.
    assert(sd_);
    sd = sd_;

    // Read the Master Boot Record (MBR) from block 0 at the very start of the medium.
    uint8_t *dataBuffer = buf.dataBuffer;
    if (!sd->read_blocks(0, dataBuffer, 1u)) {
      if (log) {
        log->println("Unable to read the MBR of the SD card");
      }
      return false;
    }

    // We require MBR, as used by manufacturers for greatest compatibility, not GPT.
    if (dataBuffer[0x1fe] != 0x55 || dataBuffer[0x1ff] != 0xaa) {
      if (log) {
        log->println("Unable to parse the MBR of the SD card");
      }
      return false;
    }

    // The MBR describes up to four primary partitions.
    uint32_t blk_offset;
    bool use_lba = true;
    bool found   = false;

    for (unsigned part = 0u; part < 1u; part++) {
      const unsigned partDesc = 0x1be + (part << 4);
      uint8_t part_type       = dataBuffer[partDesc + 4];
      uint32_t lba_start      = read32le(&dataBuffer[partDesc + 8]);
      uint32_t num_secs       = read32le(&dataBuffer[partDesc + 12]);
      uint16_t start_c, end_c;
      uint8_t start_h, end_h;
      uint8_t start_s, end_s;
      read_chs(start_c, start_h, start_s, &dataBuffer[partDesc + 1]);
      read_chs(end_c, end_h, end_s, &dataBuffer[partDesc + 5]);
      if (log) {
        log->println("Partition {} : type {} : start C {} H {} S {} : end C {} H {} S {}", part, part_type, start_c,
                     start_h, start_s, end_c, end_h, end_s);
        log->println("   LBA start: {:#010x} sectors: {:#010x}", lba_start, num_secs);
      }
      switch (part_type) {
        // Only FAT32 partitions (with or without LBA) are supported.
        case 0x0B:
          use_lba = false;
          // no break
        case 0x0C: {
          const uint16_t nheads = 255u;
          const uint16_t nsecs  = 63u;
          if (use_lba) {
            blk_offset = lba_start;
          } else {
            blk_offset = chs_to_lba(start_c, start_h, start_s, nheads, nsecs);
          }
          if (log) {
            log->println("Expecting EBR at block {:#x}", blk_offset);
          }
          found = true;
        } break;
        default:
          if (log) {
            log->println("Not a suitable partition");
          }
          break;
      }
    }

    if (!found) {
      if (log) {
        log->println("Unable to locate a suitable partition");
      }
      return false;
    }

    // Read the EBR at the start of the partition.
    if (log) {
      log->println("Reading block {}", blk_offset);
    }
    sd->read_blocks(blk_offset, dataBuffer, 1u);
    if (log) {
      dump_bytes(*log, dataBuffer, kBlockLen);
    }

    uint16_t bytesPerSector = read16le(&dataBuffer[0xb]);
    uint8_t secsPerCluster  = dataBuffer[0xd];
    uint16_t resvdSectors   = read16le(&dataBuffer[0xe]);
    uint8_t numFATs         = dataBuffer[0x10];
    uint32_t secsPerFAT     = read32le(&dataBuffer[0x24]);
    rootCluster             = read32le(&dataBuffer[0x2c]);

    if (log) {
      log->println("FAT32 {} FATs, secs per FAT {}, bytes/sec {}", numFATs, secsPerFAT, bytesPerSector);
      log->println(" resvdSectors {}", resvdSectors);
    }

    bytesPerSectorShift = floor_log2(bytesPerSector);
    secsPerClusterShift = floor_log2(secsPerCluster);

    uint32_t fatOffset         = resvdSectors;
    uint32_t clusterHeapOffset = ((resvdSectors + (numFATs * secsPerFAT)) << bytesPerSectorShift) / kBlockLen;

    // TODO: we do not fully cope with a difference between blocks and sectors at present.
    blksPerClusterShift = secsPerClusterShift;

    // Remember the volume-relative block numbers at which the (first) FAT, the cluster heap and
    // the root directory commence.
    rootStart = ((rootCluster - 2) << secsPerClusterShift << bytesPerSectorShift) / kBlockLen;
    rootStart += blk_offset + clusterHeapOffset;
    clusterHeapStart = blk_offset + clusterHeapOffset;
    fatStart         = blk_offset + fatOffset;

    if (log) {
      log->println("Cluster heap offset {} Root cluster {} log2(bytes/sec) {} log2(secs/cluster) {}", clusterHeapOffset,
                   rootCluster, bytesPerSectorShift, secsPerClusterShift);
    }

    // Sanity check the parameters, listing all objections.
    partValid = true;
    if (bytesPerSectorShift < 9 || bytesPerSectorShift > 12) {
      if (log) {
        log->println(" - bytes/sector is invalid");
      }
      partValid = false;
    }
    if (secsPerClusterShift > 25 - bytesPerSectorShift) {
      if (log) {
        log->println(" - sectors/cluster is invalid");
      }
      partValid = false;
    }
    if (!partValid) {
      if (log) {
        log->println("Unable to use this partition");
      }
      return false;
    }

    // Calculate derived properties.
    clusterBytes = 1u << (secsPerClusterShift + bytesPerSectorShift);
    clusterMask  = clusterBytes - 1u;

    // Record the fact that we have a valid partition.
    partValid = true;
    // We should now have access to the root directory when required.
    return true;
  }

  // Finalise access to a filesystem.
  void fin() {
    // Forget all files.
    for (unsigned idx = 0u; idx < kMaxFiles; idx++) {
      files[idx].flags = 0u;
    }
    // Forget all directories.
    for (unsigned idx = 0u; idx < kMaxDirs; idx++) {
      dirs[idx].flags = 0u;
    }
    // Forget all cached blocks.
    for (unsigned idx = 0u; idx < kCacheEntries; idx++) {
      blockCache[idx].block = kInvalidBlock;
    }
    blockCacheNext = 0u;
    // Forget the medium itself.
    partValid = false;
  }

  // Return the block number corresponding to the given byte offset within the specified cluster
  // of the file system, or UINT32_MAX if invalid.
  uint32_t block_number(uint32_t cluster, uint32_t offset) {
    // TODO: clusterCount not yet available.
    //    assert(cluster >= 2u && cluster < clusterCount);
    offset >>= kBytesPerBlockShift;
    return clusterHeapStart + ((cluster - 2u) << blksPerClusterShift) + offset;
  }

  // Validate directory handle.
  inline bool dh_valid(dirHandle dh) { return dh < kMaxDirs && (dirs[dh].flags & Flag_Valid); }

  // Validate file handle.
  inline bool fh_valid(fileHandle fh) { return fh < kMaxFiles && (files[fh].flags & Flag_Valid); }

  // Get a handle to the root directory of the mounted partition.
  dirHandle rootdir_open() {
    if (!partValid) {
      return kInvalidDirHandle;
    }
    return dir_open(rootCluster);
  }

  // Open a directory object that started in the given cluster.
  dirHandle dir_open(uint32_t cluster) {
    // Ensure that we have a directory handle available
    dirHandle dh = 0u;
    while (dirs[dh].flags & Flag_Valid) {
      if (++dh >= kMaxDirs) {
        return kInvalidDirHandle;
      }
    }
    // Initialise directory state.
    dirs[dh].flags        = Flag_Valid;
    dirs[dh].offset       = 0u;
    dirs[dh].length       = ~0u;  // A special directory entry marks its end.
    dirs[dh].currCluster  = cluster;
    dirs[dh].firstCluster = cluster;
    return dh;
  }

  // Return the next object within a directory, including optionally the full name of the object
  // (LFN support). If 'ucs' is null then the UCS-2 name is not returned.
  //
  // The returned characters are UCS-2 (not ASCII bytes) and a Long FileName may consist of up to
  // 255 UCS-2 characters.
  bool dir_next(dirHandle dh, dirEntry &entry, dirFlags flags = DirFlags_Default, uint16_t *ucs = nullptr,
                size_t ucs_max = 0u) {
    if (!dh_valid(dh)) {
      return false;
    }

    uint8_t entryType;
    bool hasLFN = false;
    do {
      dirEntryFlags entryFlags = dirEntryFlags(0u);
      uint8_t dir_entry[0x20u];
      if (sizeof(dir_entry) != object_read(dirs[dh], dir_entry, sizeof(dir_entry))) {
        return false;
      }
      if (log) {
        log->println("Dir entry:");
        dump_bytes(*log, dir_entry, sizeof(dir_entry));
      }
      entryType = dir_entry[0];

      uint8_t attribs = dir_entry[0xb];

      // Are we required to return this entry?
      // - _Raw demands absolutely no processing; _even_ the end of directory entry is returned.
      //
      // Ordinarily Deleted/Hidden files will be skipped, but the following flags override that
      // behaviour:
      // - _IncludeDeleted
      // - _IncludeHidden

      // Collect entry flags;
      if (hasLFN) entryFlags = dirEntryFlags(entryFlags | DirEntryFlag_HasLongName);
      if (attribs & 0x08) entryFlags = dirEntryFlags(entryFlags | DirEntryFlag_VolumeLabel);
      if (attribs & 0x010) entryFlags = dirEntryFlags(entryFlags | DirEntryFlag_Subdirectory);
      if (entryType == 0xe5) entryFlags = dirEntryFlags(entryFlags | DirEntryFlag_Deleted);

      bool entryWanted = true;
      if (!(flags & DirFlag_Raw)) {
        if (attribs == 0x0fu) {
          // Collect any Long FileName prefix entries.
          if (ucs) {
            // The sequence number allows us to calculate the offset within the buffer.
            uint8_t seqNumber = (entryType & 0x1fu);
            if (seqNumber >= 0x01 && seqNumber <= 0x14u) {
              // Each entry that forms part of the LFN contributes 13 UCS-2 characters, except the
              // final one logically (physically first in the directory object) which may include
              // a '0x0000' terminator.
              uint16_t offset = (seqNumber - 1) * 13;
              if (offset < ucs_max) {
                uint8_t lastLogical = (entryType & 0x40u);
                // Names are limited to 256 characters including the terminator.
                size_t len = (lastLogical && seqNumber >= 0x14u) ? 9 : 13;
                if (offset + len > ucs_max) {
                  len = ucs_max - offset;
                }
                // The UCS-2 name portion is scattered throughout the directory entry for
                // compatibility with earlier systems.
                copy_bytes((uint8_t *)&ucs[offset], &dir_entry[1], ((len >= 5) ? 5 : len) * 2);
                if (len > 5) {
                  copy_bytes((uint8_t *)&ucs[offset + 5], &dir_entry[0xe], ((len >= 11) ? 6 : (len - 5)) * 2);
                  if (len > 11) {
                    copy_bytes((uint8_t *)&ucs[offset + 11], &dir_entry[0x1c], (len - 11) * 2);
                  }
                }
                // Ensure that the returned name is NUL-terminated if there is space.
                if (lastLogical && (ucs_max - offset > len)) {
                  ucs[offset + len] = 0;
                }
              }
            }
          }
          // The LFN entries prefix the regular entry for a given object.
          hasLFN      = true;
          entryWanted = false;
        } else {
          entryWanted = entryType && (entryType != 0x2e) && include_entry(entryFlags, flags);
          if (!entryWanted) {
            // After a regular object that is rejected, reset the LFN flag for the following object.
            hasLFN = false;
          }
        }
      }
      if (entryWanted) {
        uint32_t cluster = ((uint32_t)read16le(&dir_entry[0x14]) << 16) | read16le(&dir_entry[0x1a]);
        // the upper nibble of the cluster must be ignored; reserved for future use.
        cluster &= ~0xf0000000u;

        entry.flags     = entryFlags;
        entry.entryType = dir_entry[0];
        // The short name of this file.
        copy_bytes(entry.shortName, dir_entry, 8);
        // File extension for the short name.
        copy_bytes(entry.shortExt, &dir_entry[8], 3);

        // Try to be helpful by reinstating the first character.
        if (entryFlags & DirEntryFlag_Deleted) entry.shortName[0] = dir_entry[0xd];
        // Also, since 0xe5 is used to mark a deleted entry, a filename that actually starts with
        // 0xe5 has historically been encoded using 0x05.
        if (entry.shortName[0] == 0x05) entry.shortName[0] = 0xe5;
        // If this object does not have a Long FileName but a buffer has been supplied, then
        // provide a conversion.
        if (ucs && !hasLFN) {
          generate_lfn(ucs, ucs_max, entry);
        }

        // See the design of the FAT file system for use/interpretation of these fields.
        entry.attribs      = dir_entry[0xb];
        entry.userAttribs  = dir_entry[0xc];
        entry.createdFine  = dir_entry[0xd];
        entry.createdTime  = read16le(&dir_entry[0xe]);
        entry.createdDate  = read16le(&dir_entry[0x10]);
        entry.accessDate   = read16le(&dir_entry[0x12]);
        entry.modifiedTime = read16le(&dir_entry[0x16]);
        entry.modifiedDate = read16le(&dir_entry[0x18]);

        // These fields are simply enough and important for file/directory access.
        entry.firstCluster = cluster;
        entry.dataLength   = read32le(&dir_entry[0x1c]);
        return true;
      }
    } while (entryType);

    return false;
  }

  // Attempt to find an extant object (file/directory) with the given name in the specified directory;
  // the search string is ASCIIZ but may be a Long FileName.
  // The UCS-2 name may be retrieved in the event of a match.
  bool dir_find(dirHandle dh, dirEntry &entry, const char *name, uint16_t *ucs = nullptr, size_t ucs_max = 0u) {
    if (!dh_valid(dh)) {
      return false;
    }
    while (dir_next(dh, entry, DirFlags_Default, buf.nameBuffer, sizeof(buf.nameBuffer) / 2)) {
      // Using the full name buffer here guarantees that 'dir_next' will have appended a NUL.
      if (!ucs2_char_compare(buf.nameBuffer, name, ~0u)) {
        if (ucs) {
          ucs2_copy(ucs, buf.nameBuffer, ucs_max);
        }
        return true;
      }
    }
    return false;
  }

  // Variant using full UCS-2 filename encoding.
  bool dir_find(dirHandle dh, dirEntry &entry, const uint16_t *ucs_name) {
    if (!dh_valid(dh)) {
      return false;
    }
    while (dir_next(dh, entry, DirFlags_Default, buf.nameBuffer, sizeof(buf.nameBuffer) / 2)) {
      // Using the full name buffer here guarantees that 'dir_next' will have appended a NUL.
      if (!ucs2_compare(buf.nameBuffer, ucs_name, ~0u)) {
        return true;
      }
    }
    return false;
  }

  // Release access to the given directory.
  void dir_close(dirHandle dh) {
    if (dh < kMaxDirs) {
      dirs[dh].flags = 0u;
    }
  }

  // Object name comparison; UCS-2 in each case. Case-sensitive matching.
  int ucs2_compare(const uint16_t *ucs1, const uint16_t *ucs2, size_t len) {
    while (len-- > 0) {
      uint16_t c2 = *ucs2++;
      uint16_t c1 = *ucs1++;
      // This handles the termination case too.
      if (!c1 || c1 != c2) {
        return (int)c1 - (int)c2;
      }
    }
    return 0;
  }

  // Object name comparison; ASCII character name against UCS-2; a convenience when matching against
  // LFN entries using an ASCIIZ name.
  int ucs2_char_compare(const uint16_t *ucs1, const char *s2, size_t len) {
    while (len-- > 0) {
      uint8_t c2  = (uint8_t)*s2++;
      uint16_t c1 = *ucs1++;
      // This handles the termination case too.
      if (!c1 || c1 != c2) {
        return (int)c1 - (int)c2;
      }
    }
    return 0;
  }

  // Utility function that copies a UCS-2 name up to an including any terminator, copying no more
  // than 'n' characters.
  void ucs2_copy(uint16_t *d, const uint16_t *s, size_t n) {
    if (n > 0u) {
      unsigned idx = 0u;
      uint16_t ch;
      do {
        ch       = s[idx];
        d[idx++] = ch;
      } while (ch && idx < n);
    }
  }

  // Open the file described by the given directory entry.
  fileHandle file_open(const dirEntry &entry) {
    // Ensure that we have a file handle available
    fileHandle fh = 0u;
    while (files[fh].flags & Flag_Valid) {
      if (++fh >= kMaxFiles) {
        return kInvalidFileHandle;
      }
    }
    // Initialise file state.
    files[fh].flags        = Flag_Valid;
    files[fh].offset       = 0u;
    files[fh].length       = entry.dataLength;
    files[fh].currCluster  = entry.firstCluster;
    files[fh].firstCluster = entry.firstCluster;
    if (log) {
      log->println("Opened file of {} byte(s) at cluster {:#10x}", entry.dataLength, entry.firstCluster);
    }
    return fh;
  }

  // Initiate read access to the given file and return a handle to the file, or InvalidFileHandle if
  // the operation is unsuccessful.
  //
  // Variants accept either ASCIIZ (char) or full UCS-2 name (uint16_t).
  template <typename T>
  fileHandle file_open(const T *name) {
    // Maintain the pretence of supporting full pathnames; they may be supported at some point.
    if (*name == '/' || *name == '\\') name++;

    fileHandle fh = kInvalidFileHandle;
    dirHandle dh  = rootdir_open();
    if (dh_valid(dh)) {
      dirEntry entry;
      if (dir_find(dh, entry, name)) {
        fh = file_open(entry);
      }
      dir_close(dh);
    }
    return fh;
  }

  // Return the length of an open file, or a negative value if the file handle is invalid.
  ssize_t file_length(fileHandle fh) { return fh_valid(fh) ? (ssize_t)files[fh].length : -1; }

  // Set the read position within a file.
  bool file_seek(fileHandle fh, uint32_t offset) {
    if (!fh_valid(fh)) {
      return 0u;
    }
    return object_seek(files[fh], offset);
  }

  // Read data from a file at the supplied offset, reading the requested number of bytes.
  size_t file_read(fileHandle fh, uint8_t *buf, size_t len) {
    if (!fh_valid(fh)) {
      return 0u;
    }
    return object_read(files[fh], buf, len);
  }

  // Return a list of clusters holding the contents of this file, starting from the current file offset,
  // and updating it upon return.
  ssize_t file_clusters(fileHandle fh, uint8_t &clusterShift, uint32_t *buf, size_t len) {
    // Check that the file handle is valid.
    if (!fh_valid(fh)) {
      return -1;
    }
    // Indicate how many blocks form a cluster for this partition.
    clusterShift = blksPerClusterShift;
    // Run forwards from the current position, permitting incremental retrieval.
    uint32_t cluster = files[fh].currCluster;
    // Ensure that the offset is aligned to the start of the cluster.
    uint32_t offset = files[fh].offset & ~clusterMask;
    size_t n        = 0u;
    while (len-- > 0u && !end_of_chain(cluster)) {
      uint32_t nextCluster;
      *buf++ = cluster;
      n++;
      if (!cluster_next(nextCluster, cluster)) {
        break;
      }
      // Remember this position within the file.
      offset += clusterBytes;
      files[fh].offset      = offset;
      files[fh].currCluster = cluster;
      cluster               = nextCluster;
    }
    return n;
  }

  // Finalise read access to the given file.
  void file_close(fileHandle fh) {
    if (fh_valid(fh)) {
      files[fh].flags = 0u;
    }
  }

  // Read Cylinder, Head and Sector ('CHS address'), as stored in a partition table entry.
  static inline void read_chs(uint16_t &c, uint8_t &h, uint8_t &s, const uint8_t *p) {
    // Head numbers are 0-based.
    h = p[0];
    // Note that sector numbers are 1-based.
    s = (p[1] & 0x3fu);
    // Cylinder numbers are 0-based.
    c = ((p[1] << 2) & 0x300u) | p[2];
  }

  // Utility function that converts Cylinder, Head, Sector (CHS) addressing into Logical Block Addressing
  // (LBA), according to the specified disk geometry.
  static uint32_t chs_to_lba(uint16_t c, uint8_t h, uint8_t s, uint8_t nheads, uint8_t nsecs) {
    // Notes: cylinder and head are zero-based but sector number is 1-based (0 is invalid).
    // CHS-addressed drives were limited to 255 heads and 63 sectors.
    if (h >= nheads || !s || s > nsecs) {
      return UINT32_MAX;
    }
    return ((c * nheads + h) * nsecs) + (s - 1);
  }

  // Read 32-bit Little Endian word.
  static inline uint32_t read32le(const uint8_t *p) {
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }

  // Read 16-bit Little Endian word.
  static inline uint16_t read16le(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }

 private:
  // We should perhaps convert to lower case only if the entire name is upper case; we do not have
  // access to a 'tolower' implementation.
  static inline uint16_t as_lower_case(uint8_t ch) { return (ch >= 'A' && ch <= 'Z') ? (ch - 'A' + 'a') : ch; }

  // Generate a Long FileName from a short form if no long form is available.
  static void generate_lfn(uint16_t *ucs, size_t ucs_max, const dirEntry &entry) {
    unsigned idx = 0u;
    // Short name.
    while (ucs_max > 0u && idx < 8u && entry.shortName[idx] > 0x20u) {
      *ucs++ = as_lower_case(entry.shortName[idx++]);
      ucs_max--;
    }
    // Period separator between short name and extension.
    if (ucs_max > 0u && entry.shortExt[0u] > 0x20u) {
      *ucs++ = '.';
      ucs_max--;
    }
    // Short extension.
    idx = 0u;
    while (ucs_max > 0u && idx < 3u && entry.shortExt[idx] > 0x20u) {
      *ucs++ = as_lower_case(entry.shortExt[idx++]);
      ucs_max--;
    }
    // NUL termination.
    if (ucs_max > 0U) {
      *ucs = 0;
    }
  }

  // Decide whether an entry with the given flags shall be returned by a directory traversal.
  static inline bool include_entry(dirEntryFlags entryFlags, dirFlags flags) {
    return (!(entryFlags & DirEntryFlag_Deleted) || (flags & DirFlag_IncludeDeleted)) &&
           (!(entryFlags & DirEntryFlag_Hidden) || (flags & DirFlag_IncludeHidden)) &&
           !(entryFlags & DirEntryFlag_VolumeLabel);
  }
};
