{
  description = "Sonata System";
  inputs = {
    lowrisc-nix.url = "github:lowRISC/lowrisc-nix";
    nixpkgs.follows = "lowrisc-nix/nixpkgs";
    flake-utils.follows = "lowrisc-nix/flake-utils";
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
      lr_doc = lowrisc-nix.lib.doc {inherit pkgs;};
      lr_pkgs = lowrisc-nix.outputs.packages.${system};
      fs = pkgs.lib.fileset;

      sonata-documentation = lr_doc.buildMdbookSite {
        inherit version;
        pname = "sonata-documentation";
        src = fs.toSource {
          root = ./.;
          fileset = lr_doc.standardMdbookFileset ./.;
        };
      };

      sonata-simulator = pkgs.stdenv.mkDerivation {
        inherit version;
        pname = "sonata-system-simulator";
        src = ./.;
        buildInputs = with pkgs; [libelf zlib];
        nativeBuildInputs = [pkgs.python311Packages.pip] ++ (with lr_pkgs; [python_ot verilator_ot]);
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
          ++ (with lr_pkgs; [
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
