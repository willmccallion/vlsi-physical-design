# Yosys at the commit OpenROAD-flow-scripts 26Q2 pins (tools/yosys), so that
# synthesis gives the netlists that flow gives. Adapted from nixpkgs' yosys
# 0.62 package, the last one built with yosys's Makefile; without the Python
# bindings and the test suite, which the flow does not use.
{
  lib,
  stdenv,
  fetchFromGitHub,
  bison,
  flex,
  pkg-config,
  libffi,
  readline,
  tcl,
  zlib,
  python3,
}:

stdenv.mkDerivation (finalAttrs: {
  pname = "yosys";
  version = "0.63";

  src = fetchFromGitHub {
    owner = "YosysHQ";
    repo = "yosys";
    rev = "d3e297fcd479247322f83d14f42b3556db7acdfb";
    hash = "sha256-HjHH1OmbDl3sfEYvymKUel1AF3TCPXNIO2+9dDbJJEI=";
    fetchSubmodules = true;
  };

  enableParallelBuilding = true;

  nativeBuildInputs = [
    bison
    flex
    pkg-config
    python3
  ];

  propagatedBuildInputs = [
    libffi
    readline
    tcl
    zlib
  ];

  makeFlags = [
    "PREFIX=${placeholder "out"}"
    "YOSYS_VER=${finalAttrs.version}"
  ];

  postPatch = ''
    substituteInPlace Makefile \
      --replace-fail 'GIT_REV := $(shell GIT_DIR=$(YOSYS_SRC)/.git git rev-parse --short=9 HEAD || echo UNKNOWN)' 'GIT_REV := v${finalAttrs.version}' \
      --replace-fail 'GIT_DIRTY := $(shell GIT_DIR=$(YOSYS_SRC)/.git git diff --exit-code --quiet 2>/dev/null; if [ $$? -ne 0 ]; then echo "-dirty"; fi)' 'GIT_DIRTY := ""'

    substituteInPlace Makefile \
      --replace-fail "| check-git-abc" ""

    substituteInPlace Makefile \
      --replace-fail 'new=$$(cd abc 2>/dev/null && git rev-parse HEAD 2>/dev/null || echo none)' 'new=none'

    sed -i 's/^YOSYS_VER :=.*/YOSYS_VER := ${finalAttrs.version}/' Makefile

    patchShebangs ./misc/yosys-config.in
  '';

  preBuild = ''
    chmod -R u+w .
    make config-${if stdenv.cc.isClang or false then "clang" else "gcc"}
  '';

  doCheck = false;

  meta = {
    description = "Open RTL synthesis framework, at the commit OpenROAD-flow-scripts 26Q2 pins";
    homepage = "https://yosyshq.net/yosys/";
    license = lib.licenses.isc;
    mainProgram = "yosys";
  };
})
