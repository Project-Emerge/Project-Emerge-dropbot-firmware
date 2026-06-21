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
        exec ${pkgs.rustc}/bin/rustc --sysroot ${rustSysroot} "$@"
      '';
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          cargo
          clang
          clippy
          espflash
          probe-rs-tools
          rust-analyzer
          rustc
          rustfmt
        ];

        LIBCLANG_PATH = "${llvm.libclang.lib}/lib";
        RUSTC_BOOTSTRAP = "1";
        RUST_SRC_PATH = "${pkgs.rustPlatform.rustLibSrc}";
      };
    };
}
