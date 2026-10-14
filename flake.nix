{
  description = "PARE: Placement And Routing Engine";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs =
    { nixpkgs, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
      rustTools =
        pkgs: with pkgs; [
          cargo
          rustc
          clippy
          rustfmt
          rust-analyzer
        ];
    in
    {
      devShells = forAllSystems (pkgs: {
        default = pkgs.mkShell {
          packages = rustTools pkgs ++ [ pkgs.python3 ];
        };
        # OpenROAD is built from source, so it lives in its own shell.
        eda = pkgs.mkShell {
          packages = rustTools pkgs ++ [
            pkgs.python3
            pkgs.openroad
          ];
        };
      });
    };
}
