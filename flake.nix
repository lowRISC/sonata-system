{
  description = "Sonata System";
  inputs = {
    lowrisc-nix.url = "github:lowRISC/lowrisc-nix";

    nixpkgs.follows = "lowrisc-nix/nixpkgs";
    flake-utils.follows = "lowrisc-nix/flake-utils";
    poetry2nix.follows = "lowrisc-nix/poetry2nix";
  };

  nixConfig = {
    extra-substituters = ["https://nix-cache.lowrisc.org/public/"];
    extra-trusted-public-keys = ["nix-cache.lowrisc.org-public-1:O6JLD0yXzaJDPiQW1meVu32JIDViuaPtGDfjlOopU7o="];
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    lowrisc-nix,
    ...
  } @ inputs: let
    system_outputs = system: let
      version = "0.0.1";

      pkgs = import nixpkgs {
        inherit system;
      };

      lrDoc = lowrisc-nix.lib.doc {inherit pkgs;};
      lrPkgs = lowrisc-nix.outputs.packages.${system};
      inherit (pkgs.lib) fileset getExe;

      pythonEnv = let
        poetry2nix = inputs.poetry2nix.lib.mkPoetry2Nix {inherit pkgs;};
        poetryOverrides = lowrisc-nix.lib.poetryOverrides {inherit pkgs;};
      in
        poetry2nix.mkPoetryEnv {
          projectDir = ./.;
          overrides = [
            poetryOverrides
            poetry2nix.defaultPoetryOverrides
          ];
        };

      sonata-documentation = lrDoc.buildMdbookSite {
        inherit version;
        pname = "sonata-documentation";
        src = fileset.toSource {
          root = ./.;
          fileset = fileset.unions [
            (lrDoc.standardMdbookFileset ./.)
            ./util/mdbook
            ./util/mdbook_wavejson.py
          ];
        };
      };

      lint-markdown = pkgs.writeShellApplication {
        name = "lint-markdown";
        text = ''
          ${getExe pkgs.lychee} --offline --include-fragments --no-progress . \
            --exclude-path ./vendor \
            --exclude-path ./build \
            --exclude-path './sw/cheri/build' \
            --exclude-path './sw/legacy/build'
        '';
      };

      cheriotPkgs = lowrisc-nix.outputs.devShells.${system}.cheriot.nativeBuildInputs;

      sonataSimulatorFileset = fileset.toSource {
        root = ./.;
        fileset = fileset.unions [
          ./rtl
          ./dv
          ./vendor
          ./sonata.core
          ./sonata_system.core
          ./rv_timer.core
          ./pulp_riscv_dbg.core
          ./open_hbmc.core
        ];
      };
      sonata-simulator-lint = pkgs.stdenvNoCC.mkDerivation {
        name = "sonta-simulator-lint";
        src = sonataSimulatorFileset;
        buildInputs = with pkgs; [libelf zlib];
        nativeBuildInputs = [pkgs.verilator pythonEnv];
        dontBuild = true;
        doCheck = true;
        checkPhase = ''
          HOME=$TMPDIR fusesoc --cores-root=. run \
            --target=lint --setup --build lowrisc:sonata:system \
            --verilator_options="+define+RVFI -j $NIX_BUILD_CORES"
        '';
        installPhase = "mkdir $out";
      };
      sonata-simulator = pkgs.stdenv.mkDerivation rec {
        inherit version;
        pname = "sonata-simulator";
        src = sonataSimulatorFileset;
        buildInputs = with pkgs; [libelf zlib];
        nativeBuildInputs = [pkgs.verilator pythonEnv];
        buildPhase = ''
          HOME=$TMPDIR fusesoc --cores-root=. run \
            --target=sim --setup --build lowrisc:sonata:system \
            --verilator_options="-j $NIX_BUILD_CORES" --make_options="-j $NIX_BUILD_CORES"
        '';
        installPhase = ''
          mkdir -p $out/bin/
          cp -r build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator $out/bin/${pname}
        '';
        meta.mainProgram = pname;
      };

      sonata-sim-boot-stub = pkgs.stdenv.mkDerivation rec {
        inherit version;
        pname = "sonata-sim-boot-stub";
        src = sw/cheri/sim_boot_stub/.;
        nativeBuildInputs = cheriotPkgs;
        buildPhase = ''
          make
        '';
        installPhase = ''
          mkdir -p $out/share/
          cp -r sim_boot_stub $out/share
        '';
      };

      cheriot-rtos-test-suite = pkgs.stdenvNoCC.mkDerivation {
        name = "cheriot-rtos-test-suite";
        src = pkgs.fetchFromGitHub {
          owner = "HU90m";
          repo = "cheriot-rtos";
          rev = "30148bad7ef45026d4e4b754871bf2b396ce2ca8";
          fetchSubmodules = true;
          hash = "sha256-3yatAEfZ5eWOfQB7w5I6OOHaG39pRUeyzCGgix3T4PI=";
        };
        buildInputs = cheriotPkgs ++ [lrPkgs.uf2conv];
        dontFixup = true;
        buildPhase = ''
          xmake config -P ./tests/ --board=sonata-prerelease
          xmake -P ./tests/
          ${./util/elf-to-uf2.sh} ./build/cheriot/cheriot/release/test-suite
        '';
        installPhase = ''
          mkdir -p $out/share/
          cp build/cheriot/cheriot/release/* $out/share/
        '';
        meta = {
          description = "The CHERIoT RTOS test suite.";
          homepage = "https://github.com/CHERIoT-Platform/cheriot-rtos/tree/main/tests";
          license = pkgs.lib.licenses.mit;
        };
      };

      tests-runner =
        pkgs.writers.writePython3Bin "tests-runner" {
          libraries = [pkgs.python3Packages.pyserial];
        }
        ./util/test_runner.py;

      tests-fpga-runner = pkgs.writers.writeBashBin "tests-fpga-runner" ''
        set -e
        if [ -z "$1" ]; then
          echo "Please provide the tty device location (e.g. /dev/ttyUSB2)" \
            "as the first argument."
          exit 2
        fi
        ${getExe tests-runner} -t 30 fpga $1 --uf2-file ${cheriot-rtos-test-suite}/share/test-suite.uf2
      '';

      tests-simulator = pkgs.stdenvNoCC.mkDerivation {
        name = "tests-simulator";
        src = ./.;
        dontBuild = true;
        doCheck = true;
        buildInputs = [sonata-simulator];
        SONATA_SIM_BOOT_STUB = "${sonata-sim-boot-stub.out}/share/sim_boot_stub";
        checkPhase = ''
          ${getExe tests-runner} -t 600 sim --elf-file ${cheriot-rtos-test-suite}/share/test-suite
        '';
        installPhase = "mkdir $out";
      };
    in {
      formatter = pkgs.alejandra;
      devShells.default = pkgs.mkShell {
        name = "sonata-system-devshell";
        packages =
          (with pkgs; [
            cmake
            screen
            picocom
            srecord
            gtkwave
            openfpgaloader
            openocd
          ])
          ++ (with lrPkgs; [
            uf2conv
            # For legacy software
            lowrisc-toolchain-gcc-rv32imcb
          ])
          ++ (with sonata-simulator; buildInputs ++ nativeBuildInputs);
      };
      packages = {inherit sonata-simulator sonata-sim-boot-stub sonata-documentation cheriot-rtos-test-suite;};
      checks = {inherit sonata-simulator-lint tests-simulator;};
      apps = builtins.listToAttrs (map (program: {
        inherit (program) name;
        value = {
          type = "app";
          program = getExe program;
        };
      }) [lint-markdown tests-runner tests-fpga-runner]);
    };
  in
    flake-utils.lib.eachDefaultSystem system_outputs;
}
