{
  description = "A flake for declaratively pulling in build dependencies on NixOS";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs =
    {
      nixpkgs,
      ...
    }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
      version = "2027.0.0-alpha-7";

      wpilibJdk =
        pkgs.runCommand "wpilib-jdk"
          {
            src = pkgs.fetchzip {
              url = "https://packages.wpilib.workers.dev/installer/v${version}/WPILib_Linux-x64-${version}.tar.gz";
              hash = "sha256-M8W9D03Hd6JVRymlPmfiXsV3FAQyJQPyzKUPQDRqZos=";
            };
          }
          ''
            mkdir -p $out

            tar -xzf $src/WPILib_Linux-${version}-artifacts.tar.gz jdk
            cp -r jdk/* $out/
          '';
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        name = "first";

        packages = [ wpilibJdk pkgs.libGL ];

        JAVA_HOME = wpilibJdk;
      };
    };
}