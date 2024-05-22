{
  description = "Sonata System";
  inputs = {
    lowrisc-nix.url = "github:lowRISC/lowrisc-nix";
    nixpkgs.follows = "lowrisc-nix/nixpkgs";
    flake-utils.follows = "lowrisc-nix/flake-utils";
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
  }: let
    system_outputs = system: let
      version = "0.0.1";

      pkgs = import nixpkgs {
        inherit system;
      };
      lrDoc = lowrisc-nix.lib.doc {inherit pkgs;};
      lrPkgs = lowrisc-nix.outputs.packages.${system};
      fs = pkgs.lib.fileset;

      sonata-documentation = lrDoc.buildMdbookSite {
        inherit version;
        pname = "sonata-documentation";
        src = fs.toSource {
          root = ./.;
          fileset = lrDoc.standardMdbookFileset ./.;
        };
      };

      sonata-simulator = pkgs.stdenv.mkDerivation {
        inherit version;
        pname = "sonata-system-simulator";
        src = ./.;
        buildInputs = with pkgs; [libelf zlib];
        nativeBuildInputs = [pkgs.python311Packages.pip] ++ (with lrPkgs; [python_ot verilator_ot]);
        buildPhase = ''
          HOME=$TMPDIR fusesoc --cores-root=. run \
            --target=sim --tool=verilator --setup \
            --build lowrisc:sonata:system \
            --verilator_options="+define+RVFI -j $NIX_BUILD_CORES" \
            --make_options="-j $NIX_BUILD_CORES"
        '';
        installPhase = ''
          mkdir -p $out/bin/
          cp -r build/lowrisc_sonata_system_0/sim-verilator/Vtop_verilator $out/bin/
        '';
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
      packages = {inherit sonata-simulator sonata-documentation;};
    };
  in
    flake-utils.lib.eachDefaultSystem system_outputs;
}
