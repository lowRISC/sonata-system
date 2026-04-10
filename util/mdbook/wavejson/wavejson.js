// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// WaveDrom must run after mdBook has finished all DOM mutations. Using `window.load`
// is unreliable because content might still be re-written at that time, causing
// WaveDROM to see a partially mutated DOM and fail. It seems likely that newer
// mdBook versions have somehow implemented heavier / longer DOM manipulation.
document.addEventListener("DOMContentLoaded", () =>
    requestAnimationFrame(() => WaveDrom.ProcessAll())
);
