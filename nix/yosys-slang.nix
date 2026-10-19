# The yosys-slang SystemVerilog frontend at the commit OpenROAD-flow-scripts
# 26Q2 pins (tools/yosys-slang), which its synthesis loads with
# `plugin -i slang` for designs such as ibex.
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  python3,
  boost,
  yosys,
}:

stdenv.mkDerivation {
  pname = "yosys-slang";
  version = "0-unstable-2026-01-02";

  src = fetchFromGitHub {
    owner = "povik";
    repo = "yosys-slang";
    rev = "64b44616a3798f07453b14ea03e4ac8a16b77313";
    hash = "sha256-kfu59/M3+IM+5ZMd+Oy4IZf4JWuVtPDlkHprk0FB8t4=";
    fetchSubmodules = true;
  };

  nativeBuildInputs = [
    cmake
    python3
    yosys
  ];

  buildInputs = [
    boost
    yosys
  ];

  cmakeFlags = [
    (lib.cmakeFeature "YOSYS_CONFIG" "${yosys}/bin/yosys-config")
  ];

  installPhase = ''
    runHook preInstall
    install -Dm755 slang.so $out/share/yosys/plugins/slang.so
    runHook postInstall
  '';

  meta = {
    description = "SystemVerilog frontend for yosys, at the commit OpenROAD-flow-scripts 26Q2 pins";
    homepage = "https://github.com/povik/yosys-slang";
    license = lib.licenses.isc;
  };
}
