#!/usr/bin/env bash
# Copyright 2018 Slightech Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(realpath "$BASE_DIR/..")

# \usepackage{CJKutf8}
# \begin{document}
# \begin{CJK}{UTF8}{gbsn}
# ...
# \end{CJK}
# \end{document}
_texcjk() {
  tex="$1"; shift;
  echo "add cjk to $tex"
  sed -i "" -E $'s/^\\\\begin{document}$/\\\\usepackage{CJKutf8}\\\n\\\\begin{document}\\\n\\\\begin{CJK}{UTF8}{gbsn}/g' $tex
  sed -i "" -E $'s/^\\\\end{document}$/\\\\end{CJK}\\\n\\\\end{document}/g' $tex
}

DOXYFILE="api.doxyfile"
OUTPUT="$BASE_DIR/_output"

_generate() {
  lang="$1"; shift;
  echo "Build doc $lang"
  cd "$BASE_DIR/$lang"
  if [ -f "$DOXYFILE" ]; then
    outdir="$OUTPUT/$lang"
    [ -e "$outdir" ] || mkdir -p "$outdir"
    echo "doxygen $DOXYFILE"
    doxygen $DOXYFILE
    if type "pdflatex" &> /dev/null && [ -f "$outdir/latex/Makefile" ]; then
      echo "doxygen make latex"
      version=`cat $DOXYFILE | grep -m1 "^PROJECT_NUMBER\s*=" | \
        sed -E "s/^.*=[[:space:]]*(.*)[[:space:]]*$/\1/g"`
      filename="mynt-eye-d-sdk-apidoc"; \
        [ -n "$version" ] && filename="$filename-$version"; \
        filename="$filename-$lang.pdf"
      cd "$outdir/latex" && _texcjk refman.tex && make
      [ -f "refman.pdf" ] && mv "refman.pdf" "../$filename"
    fi
    echo "doxygen completed"
  else
    echo "$DOXYFILE not found"
  fi
}

source "$BASE_DIR/langs.sh"

for lang in "${LANGS[@]}"; do
  [ -d "$BASE_DIR/$lang" ] || continue
  _generate "$lang"
done

cd "$BASE_DIR"
