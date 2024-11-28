/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * This is a `lorem ipsum` (https://en.wikipedia.org/wiki/Lorem_ipsum) text
 * used to test reading from the microSD card. The text can be emitted over
 * the UART by setting `emitText` to true, captured and stored as `LOREM.IPS`
 * in the root directory of a FAT32-formatted microSD card.
 *
 * The data read from the file on the microSD card is then compared with this
 * text.
 */
static const char lorem_text[] = {
    "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sociosqu "
    "consectetur, tempor nisl si rutrum nibh. Ullamcorper iaculis ornare mauris "
    "eleifend, eu convallis porttitor pharetra nisi. Nullam condimentum "
    "tincidunt, vulputate facilisi, maecenas tortor. Eu convallis, feugiat "
    "facilisis per magna venenatis. Sodales natoque, lectus tristique aptent "
    "scelerisque, ac sociis ligula. Augue nisl torquent magnis, mi platea "
    "eleifend suspendisse. Morbi dapibus montes mattis, magna do sociis posuere. "
    "Natoque, taciti volutpat porttitor, ultricies amet. Sapien varius euismod, "
    "dignissim ad sociis, molestie maximus phasellus."
    "\r\n"
    "\r\n"
    "Feugiat, sociosqu parturient fringilla, do aliquet facilisis quisque "
    "vulputate. Elit vitae sagittis sapien mattis, at phasellus blandit "
    "consectetur ligula dictumst tortor. Proin dignissim suspendisse, lectus ad "
    "natoque, interdum libero augue. Scelerisque lacinia mauris morbi. Feugiat "
    "sagittis proin iaculis si augue fermentum. Sapien lectus euismod, lorem "
    "suspendisse, justo sociis. Arcu ultrices commodo amet, sociosqu rutrum, "
    "facilisis mus convallis. Neque condimentum nisl orci dolor, si mattis sed "
    "magnis. Proin lorem, vulputate fusce, id feugiat adipiscing commodo aliquet. "
    "Tortor, litora natoque lacinia, mattis posuere ullamcorper vitae. Dictumst "
    "lectus imperdiet, consectetur porttitor maecenas gravida. Condimentum "
    "tristique ac natoque nascetur praesent rutrum. Mauris aliquam fringilla, "
    "per gravida eget, eu scelerisque. Praesent egestas cursus class condimentum, "
    "mi mattis posuere tempor semper ridiculus vulputate."
    "\r\n"
    "\r\n"
    "Aliquet euismod ante pellentesque, gravida tincidunt, per luctus morbi. "
    "Varius, montes magna pulvinar, molestie eu tempor. Platea pharetra laoreet, "
    "ut viverra lacinia, mus dignissim. Mattis suspendisse luctus gravida, "
    "penatibus per tempor nisl. Torquent arcu porttitor nec iaculis, at vitae "
    "posuere condimentum lacinia nostra aliquet. Parturient, praesent penatibus "
    "adipiscing, fusce duis consectetur. Justo feugiat porttitor, tempor vivamus, "
    "turpis rutrum. Parturient facilisi potenti consectetur, natoque nibh, vel "
    "ullamcorper iaculis. Taciti feugiat, lorem ipsum, non vehicula lectus cursus. "
    "Dolor, urna convallis lacinia, in natoque interdum, enim vulputate. Justo "
    "varius nisl, pharetra imperdiet, at velit tortor. Platea morbi inceptos, "
    "volutpat laoreet, ut vehicula aliquam. Penatibus risus, elit gravida erat "
    "ullamcorper condimentum. Odio dolor, vulputate imperdiet ad eleifend mollis."
    "\r\n"
    "\r\n"
    "Arcu, elit tempor cursus, vel gravida sed, litora lorem. Tortor, nulla "
    "parturient sollicitudin, at dolor nascetur. Elit penatibus interdum, "
    "pellentesque tristique orci iaculis, per convallis. Gravida, litora amet "
    "efficitur, si vitae ultricies, mus lorem. Torquent mattis posuere, vulputate "
    "ligula, eu dictum scelerisque. Cras consectetur sagittis, magnis pulvinar "
    "felis volutpat, do parturient. Ante aliquet venenatis, gravida fusce, purus "
    "pellentesque. Habitasse condimentum, eleifend euismod, ac magna sagittis mus "
    "mattis. Parturient, ante pharetra facilisis, erat litora aliquam. Aliquet "
    "bibendum etiam pellentesque, si magnis, himenaeos vulputate blandit. "
    "Parturient nibh, turpis volutpat interdum congue morbi. Pharetra, orci amet "
    "fermentum, magnis nascetur, do vehicula scelerisque. Odio parturient posuere "
    "dis aliquet, mi aliquam ligula augue. Mus mattis vivamus, rutrum at "
    "vulputate, suspendisse orci lorem."
    "\r\n"
    "\r\n"
    "Habitasse morbi, pharetra venenatis dictum tempor, eu iaculis tristique. Dis "
    "vulputate, convallis blandit, gravida bibendum mus volutpat. Eleifend "
    "efficitur habitasse dolor, a vitae porttitor ullamcorper lorem. Rutrum "
    "venenatis maximus egestas, orci augue cursus. Purus pellentesque, tempor "
    "lacinia, per eleifend erat neque consectetur. Dolor, eu aliquet fusce, si "
    "phasellus fringilla sapien amet. Eleifend, sociosqu penatibus ultrices, ac "
    "scelerisque euismod. Massa vitae, arcu sollicitudin, et ante parturient "
    "lacinia. Sapien phasellus interdum, condimentum semper, tellus gravida. Orci "
    "feugiat bibendum congue penatibus, mi morbi nisl volutpat imperdiet praesent "
    "convallis. Gravida tristique curabitur pellentesque, at vulputate lacinia "
    "mauris varius interdum eleifend. Tempor tincidunt odio penatibus, do "
    "ridiculus phasellus ultrices."
    "\r\n"
    "\r\n"};
