#!/usr/bin/env python
# Disable pylint filename and missing module member complaints.
# pylint: disable=C0103,E1101

"""
Initializes git hooks for the parent git repository.

TODO(mfehr): fix this script such that we can use the new cpplint. It currently
results in warnings that are not correct IMO. Also ideally find a way to use
the cpplint file online directly, however that would require a different way
of parsing the linter output.

Set this variable to download the cpplint file instead of using the
local !modified! copy:

cpplint_url = "https://raw.githubusercontent.com/google/styleguide/gh-pages/cpplint/cpplint.py"

Set this variable to use the local modified copy of the newest cpplint script:

default_cpplint = "cpplint.py"
default_pylint = "pylint.rc"
"""

import os
import requests
import shutil
import subprocess
import sys

default_cpplint = "modified_cpplint.py"
default_pylint = "modified_pylint.rc"

cpplint_url = ""
pylint_url = ""
# pylint_url = "https://raw.githubusercontent.com/vinitkumar/googlecl/6dc04b489dba709c53d2f4944473709617506589/googlecl-pylint.rc"

clang_format_diff_executable = "clang-format-diff"


def download_file_from_url(url, file_path):
  """Download a file from a HTTPS URL. Verification is enabled."""
  request = requests.get(url, verify=True, stream=True)
  request.raw.decode_content = True
  with open(file_path, 'w') as downloaded_file:
    shutil.copyfileobj(request.raw, downloaded_file)


def get_git_repo_root(some_folder_in_root_repo='./'):
  """Get the root folder of the git repository."""
  get_repo_call = subprocess.Popen("git rev-parse --show-toplevel",
                                   shell=True,
                                   cwd=some_folder_in_root_repo,
                                   stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE)

  stdout, _ = get_repo_call.communicate()
  repo_root = stdout.rstrip()
  return repo_root


def command_exists(cmd):
  """Check if a bash command exists."""
  return subprocess.call("type " + cmd, shell=True,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE) == 0


def parse_args():
  import argparse
  parser = argparse.ArgumentParser(
      prog=os.path.basename(__file__),
      formatter_class=argparse.RawTextHelpFormatter,
      description='usage examples:'
      '\n  python %(prog)s -hr HOOKS_ROOT')
  parser.add_argument(
      '-hr',
      '--hooks-root',
      dest='hooks_root',
      metavar='HOOKS_ROOT',
      required=False,
      help='the hooks root folder')
  return parser.parse_args()


def main():
  """ Download cpplint.py and pylint.py and installs the git hooks"""
  script_directory = os.path.dirname(sys.argv[0])
  script_directory = os.path.abspath(script_directory)

  if cpplint_url != "":
    # Download linter files.
    download_file_from_url(cpplint_url, script_directory + "/cpplint.py")
    if not os.path.isfile(script_directory + "/cpplint.py"):
      print("ERROR: Could not download cpplint.py file!")
      exit(1)
  else:
    cp_params = (script_directory + "/default/" + default_cpplint + " " +
                 script_directory + "/cpplint.py")
    if subprocess.call("cp " + cp_params, shell=True) != 0:
      print("Failed to copy default cpplint.")
      exit(1)

  if pylint_url != "":
    download_file_from_url(pylint_url, script_directory + "/pylint.rc")
    if not os.path.isfile(script_directory + "/pylint.rc"):
      print("ERROR: Could not download pylint.rc file!")
      exit(1)
  else:
    cp_params = (script_directory + "/default/" + default_pylint + " " +
                 script_directory + "/pylint.rc")
    if subprocess.call("cp " + cp_params, shell=True) != 0:
      print("Failed to copy default pylint.")
      exit(1)

  if not command_exists(clang_format_diff_executable):
    print("ERROR: " + clang_format_diff_executable + " is not installed!")
    exit(1)

  if not command_exists("autopep8"):
    print("ERROR: autopep8 is not installed! Try: pip install autopep8")
    exit(1)

  args = parse_args()
  if args.hooks_root:
    hooks_root = args.hooks_root
  else:
    # Get git root folder of parent repository.
    repo_root = get_git_repo_root(script_directory + '/../')
    hooks_root = repo_root.decode() + "/.git/hooks"

  # Copy git hooks.
  cp_params = script_directory + "/git-hooks.py " + \
      hooks_root + "/pre-commit"
  if subprocess.call("cp " + cp_params, shell=True) != 0:
    print("Failed to copy githooks to {}/...".format(hooks_root))
    exit(1)

  print("Success, githooks initialized!")


if __name__ == "__main__":
  main()
