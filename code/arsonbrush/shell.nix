{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {

    buildInputs = with pkgs; [
        gcc-arm-embedded
        gnumake
        gdb
        pkg-config
        stlink
    ];

    shellHook = ''
        echo "Ready"
    '';
}