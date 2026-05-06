{
  description = "A very basic flake";

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
      version = "2027.0.0-alpha-2";

      wpilibJdk =
        pkgs.runCommand "wpilib-jdk"
          {
            src = pkgs.fetchzip {
              url = "https://packages.wpilib.workers.dev/installer/v${version}/Linux/WPILib_Linux-${version}.tar.gz";
              hash = "sha256-deHmTM7tX1TpDFVVIosKsi07fYLwZ76jVPW2JR/6jwU=";
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
        name = "relic";

        packages = [ wpilibJdk pkgs.libGL ];

        JAVA_HOME = wpilibJdk;
      };
    };
}
