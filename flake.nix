{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    utils.url = "github:numtide/flake-utils";
    pros-cli-nix.url = "github:BattleCh1cken/pros-cli-nix";
  };

  outputs = {
    self,
    nixpkgs,
    utils,
    pros-cli-nix,
  }:
    utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {inherit system;};
    in {
      devShell = pkgs.mkShell {
        packages = with pkgs; [
          pros-cli-nix.packages.${system}.default
          gcc-arm-embedded
          clang
        ];
        shellHook = ''
          clear
               echo -n Bobot go brrrr
               export MAKEFLAGS="-j $((`nproc` - 1))"
               alias mut="pros --no-sentry --no-analytics mut --after run"
               alias mu="pros --no-sentry --no-analytics mu"
               alias m="pros --no-sentry --no-analytics build-compile-commands"
               alias t="pros --no-sentry --no-analytics t"
        '';
      };
    });
}
