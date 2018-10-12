# linter

This repo contains a (C++, python (experimental)) linter and auto formatter package that can be included into your repository as a submodule. It provides the following git hooks:
 * **General**
   * Prevent commits to master.
 * **C++** files:
   * **clang-format** Formats your code based on your .clang-format preferences.
   * **cpplint** Checks your C++ code for style errors and warnings.

 * **EXPERIMENTAL - Python** files:

      **WARNING:** The default linter settings are very strict and might not actually conform with the formatting produced by autopep8. If you want to use this linter for python you will need to do some tweaking.
      * **autopep8** Formats your python code.
      * **pylint** Checks your Python code for style errors and warnings.


## Dependencies

 * **autopep8** ([Introduction to autopep8](http://avilpage.com/2015/05/automatically-pep8-your-python-code.html))
   * Ubuntu / macOS: `pip install autopep8`
 * **clang-format**
   * Ubuntu: `sudo apt-get install clang-format`
   * macOS:
     ```
     brew install clang-format
     ln -s /usr/local/share/clang/clang-format-diff.py /usr/local/bin/clang-format-diff
     ```


## Installation

```bash
cd $YOUR_REPO
git submodule add -b slightech https://github.com/slightech/linter.git
./linter/init-git-hooks.py
```

You can also add the linter submodule in a subfolder of your repo, e.g.:
```bash
mkdir $YOUR_REPO/tools
git submodule add -b slightech https://github.com/slightech/linter.git tools/linter
./tools/linter/init-git-hooks.py
```

Define the project-specific C++ format by adding a file `.clang-format` to your projects root folder. Example:

```
---
Language: Cpp
BasedOnStyle: Google
DerivePointerAlignment: false
PointerAlignment: Left
ColumnLimit: 80
AllowShortFunctionsOnASingleLine: Empty
AllowShortIfStatementsOnASingleLine: false
AllowShortLoopsOnASingleLine: false
AlignAfterOpenBracket: AlwaysBreak
IncludeCategories:
  - Regex:           '^<.*'
    Priority:        1
  - Regex:           '.*'
    Priority:        2
...
```

#### ASCII-Art Sources

 * [www.retrojunkie.com (accessed through web.archive.org)](https://web.archive.org/web/20150831003349/http://www.retrojunkie.com:80/asciiart/)
