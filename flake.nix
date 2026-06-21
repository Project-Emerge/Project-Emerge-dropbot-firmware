{
  description = "Project Emerge Dropbot firmware";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
      llvm = pkgs.llvmPackages_latest;
      rustSysroot = pkgs.symlinkJoin {
        name = "rust-sysroot";
        paths = [ pkgs.rustc-unwrapped ];
        postBuild = ''
          mkdir -p "$out/lib/rustlib/src/rust"
          ln -s ${pkgs.rustPlatform.rustLibSrc} "$out/lib/rustlib/src/rust/library"
        '';
      };
      rustc = pkgs.writeShellScriptBin "rustc" ''
        extra_sysroot=(--sysroot ${rustSysroot})
        for arg in "$@"; do
          case "$arg" in
            --sysroot|--sysroot=*)
              extra_sysroot=()
              break
              ;;
          esac
        done

        exec ${pkgs.rustc}/bin/rustc "''${extra_sysroot[@]}" "$@"
      '';
      clippyDriver = pkgs.writeShellScriptBin "clippy-driver" ''
        extra_sysroot=(--sysroot ${rustSysroot})
        for arg in "$@"; do
          case "$arg" in
            --sysroot|--sysroot=*)
              extra_sysroot=()
              break
              ;;
          esac
        done

        if [ "$#" -gt 0 ] && [ "$(basename "$1")" = "rustc" ]; then
          rustc_cmd="$1"
          shift
          exec ${pkgs.clippy}/bin/clippy-driver "$rustc_cmd" "''${extra_sysroot[@]}" "$@"
        fi

        exec ${pkgs.clippy}/bin/clippy-driver "''${extra_sysroot[@]}" "$@"
      '';
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          cargo
          clang
          clippyDriver
          clippy
          espflash
          lld
          probe-rs-tools
          rust-analyzer
          rustc
          rustfmt
        ];

        LIBCLANG_PATH = "${llvm.libclang.lib}/lib";
        RUSTC_BOOTSTRAP = "1";
        RUSTFLAGS = "--sysroot ${rustSysroot}";
        RUST_SRC_PATH = "${pkgs.rustPlatform.rustLibSrc}";
      };
    };
}
