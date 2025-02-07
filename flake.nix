{
  description = "A Nix-flake-based development environment";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-24.11";
    crPkgs.url = "github:chai-yuan/crpkgs";
  };

  outputs =
    {
      self,
      nixpkgs,
      crPkgs,
    }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
      crpkgs = crPkgs.packages.${system};
    in
    {
      devShells."${system}".default = pkgs.mkShell {
        packages = [
          crpkgs.riscv64-elf-gcc
          pkgs.pkgsCross.riscv64.buildPackages.gcc
          pkgs.qemu
          pkgs.spike
          pkgs.gcc
          pkgs.gnumake
          pkgs.bear
          pkgs.clang-tools
          pkgs.gdb
          pkgs.dtc
          pkgs.flex
          pkgs.bison
          pkgs.pkg-config
          pkgs.ncurses
          pkgs.boost
          pkgs.asio
        ];

        shellHook = ''
          echo "A Nix-flake-based development environment"
        '';
      };
    };
}
