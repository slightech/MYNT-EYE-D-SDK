#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Disable pylint invalid module name complaint.
# pylint : disable = C0103

""" Pre-commit script for invoking linters and auto-formatters."""

# Make sure we use the new print function
from __future__ import print_function
from random import randint

import datetime
import imp
import os
import pylint.lint
import re
import subprocess

CLANG_FORMAT_DIFF_EXECUTABLE = "clang-format-diff"
AUTOPEP8_FORMAT_EXECUTABLE = "autopep8"


def run_command_in_folder(command, folder):
  """Run a bash command in a specific folder."""
  run_command = subprocess.Popen(command,
                                 shell=True,
                                 cwd=folder,
                                 stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE)
  stdout, _ = run_command.communicate()
  command_output = stdout.rstrip()
  return command_output.decode()


def get_number_of_commits(some_folder_in_root_repo='./'):
  command = "git shortlog -s -n --all --author=\"$(git config --get user.email)\" | grep -o '[0-9]\+' |  awk '{s+=$1} END {print s}'"
  num_commits = run_command_in_folder(command, some_folder_in_root_repo)
  if not num_commits:
    num_commits = "0"
  return num_commits


def get_staged_files(some_folder_in_root_repo='./'):
  """Get all staged files from git."""
  output = run_command_in_folder(
      "git diff --staged --name-only", some_folder_in_root_repo)
  if len(output) == 0:
    return []
  else:
    return output.split("\n")


def get_unstaged_files(some_folder_in_root_repo='./'):
  """Get all unstaged files from git."""
  output = run_command_in_folder(
      "git diff --name-only", some_folder_in_root_repo)
  return output.split("\n")


def check_cpp_lint(staged_files, cpplint_file, ascii_art, repo_root):
  """Runs Google's cpplint on all C++ files staged for commit,"""
  cpplint = imp.load_source('cpplint', cpplint_file)
  cpplint._cpplint_state.SetFilters(
      '-legal/copyright,-build/c++11')  # pylint: disable=W0212

  # Prevent cpplint from writing to stdout directly, instead
  # the errors will be stored in pplint.output as (line, message) tuples.
  cpplint.print_stdout = False

  total_error_count = 0
  for changed_file in staged_files:
    if not os.path.isfile(changed_file):
      continue
    if changed_file.lower().endswith(('.cc', '.h', '.cpp', '.cu', '.cuh')):
      # Search iteratively for the root of the catkin package.
      package_root = ''
      search_dir = os.path.dirname(os.path.abspath(changed_file))
      found_package_root = False
      MAX_DEPTH_OF_FILES = 100
      for _ in range(1, MAX_DEPTH_OF_FILES):
        if os.path.isfile(search_dir + '/package.xml'):
          package_root = search_dir
          found_package_root = True
          break
        # Stop if the root of the git repo is reached.
        if os.path.isdir(search_dir + '/.git'):
          package_root = search_dir
          found_package_root = True
          break
        search_dir = os.path.dirname(search_dir)
      assert found_package_root, ("Could not find the root of the "
                                  "catkin package that contains: "
                                  "{}".format(changed_file))

      cpplint._root = package_root + '/include'   # pylint: disable=W0212

      # Reset error count and messages:
      cpplint.output = []
      cpplint._cpplint_state.ResetErrorCounts()   # pylint: disable=W0212
      v_level = cpplint._cpplint_state.verbose_level  # pylint: disable=W0212

      cpplint.ProcessFile(changed_file, v_level)

      error_count = cpplint._cpplint_state.error_count  # pylint: disable=W0212
      if error_count > 0:
        total_error_count += error_count

        print("-" * 80)
        print("Found {} errors in : {}".format(
            error_count, changed_file))
        print("-" * 80)
        for line in cpplint.output:
          assert len(line) == 2
          print("line {: >4}:\t {}".format(line[0], line[1]))

  if total_error_count > 0:
    print("=" * 80)
    print("Found {} cpplint errors".format(total_error_count))
    print("=" * 80)

    if total_error_count > 50:
      print(ascii_art.AsciiArt.cthulhu)
    elif total_error_count > 20:
      print(ascii_art.AsciiArt.tiger)
    return False
  else:
    return True


def check_modified_after_staging(staged_files):
  """Checks if one of the staged files was modified after staging."""
  files_changed = get_unstaged_files()
  files_changed = filter(None, files_changed)  # pylint: disable=W0141

  staged_files_changed = 0

  staged_files_changed_list = []

  is_first = True
  for changed_file in files_changed:
    if changed_file in staged_files:

      if is_first:
        print("=" * 80)
        is_first = False

      print("\'{}\' was modified after staging".format(changed_file))
      staged_files_changed_list.append(changed_file)
      staged_files_changed += 1

  if staged_files_changed > 0:
    print("-" * 80)
    print("Found {} files that have been changed after staging".format(
        staged_files_changed))
    print("=" * 80)

  return staged_files_changed_list


def check_commit_against_master(repo_root):
  """Check if the current commit is intended to for the master branch."""
  current_branch = run_command_in_folder(
      "git branch | grep \"*\" | sed \"s/* //\"", repo_root)
  print("\nCurrent_branch: {}\n".format(current_branch))
  return current_branch == "master"


def check_if_merge_commit(repo_root):
  """Check if the current commit is a merge commit."""
  merge_msg_file_path = repo_root + "/.git/MERGE_MSG"
  return os.path.isfile(merge_msg_file_path)


def run_clang_format(repo_root, staged_files, list_of_changed_staged_files):
  """Runs clang format on all cpp files staged for commit."""

  clang_format_path = ("/tmp/" + os.path.basename(
      os.path.normpath(repo_root)) + "_" +
      datetime.datetime.now().isoformat() + ".clang.patch")

  run_command_in_folder("git diff -U0 --cached | " +
                        CLANG_FORMAT_DIFF_EXECUTABLE +
                        " -style=file -p1 > " +
                        clang_format_path, repo_root)

  if not os.stat(clang_format_path).st_size == 0:
    if list_of_changed_staged_files:
      print("=" * 80)
      print("Cannot format your code, because some files are \n"
            "only partially staged! Format your code or try \n"
            "stashing your unstaged changes...")
      print("=" * 80)
      exit(1)

    run_command_in_folder("git apply -p0 " + clang_format_path, repo_root)
    run_command_in_folder("git add " + ' '.join(staged_files), repo_root)

    print("=" * 80)
    print("Formatted staged C++ files with clang-format.\n" +
          "Patch: {}".format(clang_format_path))
    print("=" * 80)
  return True


def run_autopep8_format(repo_root, staged_files, list_of_changed_staged_files):
  """Runs autopep8 format on all python files staged for commit."""

  first_file_formatted = True
  for staged_file in staged_files:
    if not os.path.isfile(staged_file):
      continue
    if staged_file.endswith((".py")):

      # Check if the file needs formatting by applying the formatting and store
      # the results into a patch file.
      autopep8_format_path = ("/tmp/" +
                              os.path.basename(os.path.normpath(repo_root)) +
                              "_" + datetime.datetime.now().isoformat() +
                              ".autopep8.patch")
      task = (AUTOPEP8_FORMAT_EXECUTABLE +
              " -d --aggressive --aggressive --max-line-length=80 " +
              "--indent-size=2 --ignore=E24 " +
              staged_file + " > " + autopep8_format_path)
      run_command_in_folder(task, repo_root)

      if not os.stat(autopep8_format_path).st_size == 0:
        if first_file_formatted:
          print("=" * 80)
          print("Formatted staged python files with autopep8:")
          first_file_formatted = False

        print("-> " + staged_file)

        if staged_file in list_of_changed_staged_files:
          print("Cannot format your code, because this file is \n"
                "only partially staged! Format your code or try \n"
                "stashing your unstaged changes...")
          print("=" * 80)
          exit(1)
        else:
          run_command_in_folder(
              "git apply -p1 " + autopep8_format_path, repo_root)
          run_command_in_folder("git add " + staged_file, repo_root)

  if not first_file_formatted:
    print("=" * 80)
  return True


def check_python_lint(repo_root, staged_files, pylint_file):
  """Runs pylint on all python scripts staged for commit."""

  class TextReporterBuffer(object):
    """Stores the output produced by the pylint TextReporter."""

    def __init__(self):
      """init"""
      self.content = []

    def write(self, input_str):
      """write"""
      self.content.append(input_str)

    def read(self):
      """read"""
      return self.content

  # Parse each pylint output line individualy and searches
  # for errors in the code.
  pylint_errors = []
  for changed_file in staged_files:
    if not os.path.isfile(changed_file):
      continue
    if re.search(r'\.py$', changed_file):

      print("Running pylint on \'{}\'".format(repo_root + "/" + changed_file))
      pylint_output = TextReporterBuffer()
      pylint_args = ["--rcfile=" + pylint_file,
                     "-rn",
                     repo_root + "/" + changed_file]
      from pylint.reporters.text import TextReporter
      pylint.lint.Run(pylint_args,
                      reporter=TextReporter(pylint_output),
                      exit=False)

      for output_line in pylint_output.read():
        if re.search(r'^(E|C|W):', output_line):
          print(changed_file + ": " + output_line)
          pylint_errors.append(output_line)

  if len(pylint_errors) > 0:
    print("=" * 80)
    print("Found {} pylint errors".format(len(pylint_errors)))
    print("=" * 80)
    return False
  else:
    return True


def linter_check(repo_root, linter_subfolder):
  """ Main pre-commit function for calling code checking script. """

  cpplint_file = repo_root + "/" + linter_subfolder + "/cpplint.py"
  pylint_file = repo_root + "/" + linter_subfolder + "/pylint.rc"
  ascii_art_file = repo_root + "/" + linter_subfolder + "/ascii_art.py"

  print("Found linter subfolder: {}".format(linter_subfolder))
  print("Found ascii art file at: {}".format(ascii_art_file))
  print("Found cpplint file at: {}".format(cpplint_file))
  print("Found pylint file at: {}".format(pylint_file))

  # Run checks
  staged_files = get_staged_files()

  if len(staged_files) == 0:
    print("\n")
    print("=" * 80)
    print("No files staged...")
    print("=" * 80)
    exit(1)

  # Load ascii art.
  ascii_art = imp.load_source('ascii_art', ascii_art_file)

  if check_commit_against_master(repo_root):
    print(ascii_art.AsciiArt.grumpy_cat)
    exit(1)

  if not check_if_merge_commit(repo_root):
    # Do not allow commiting files that were modified after staging.This
    # avoids problems such as forgetting to stage fixes of cpplint complaints.
    list_of_changed_staged_files = check_modified_after_staging(staged_files)

    run_clang_format(repo_root, staged_files, list_of_changed_staged_files)
    run_autopep8_format(repo_root, staged_files, list_of_changed_staged_files)

    # Use Google's C++ linter to check for compliance with Google style guide.
    cpp_lint_success = check_cpp_lint(
        staged_files, cpplint_file, ascii_art, repo_root)

    # Use pylint to check for comimpliance with Tensofrflow python style guide.
    pylint_success = check_python_lint(repo_root, staged_files, pylint_file)

    if not(cpp_lint_success and pylint_success):
      print("=" * 80)
      print("Commit rejected! Please address the linter errors above.")
      print("=" * 80)
      exit(1)
    else:

      commit_number = get_number_of_commits(repo_root)
      lucky_commit = ((int(commit_number) + 1) % 42 == 0)

      if lucky_commit:
        print(ascii_art.AsciiArt.story)

        print("=" * 80)
        print("Commit accepted, well done! This is your {}th commit!".format(
            commit_number))
        print("This is a lucky commit! Please enjoy this free sheep story.")
        print("=" * 80)
      else:
        print(ascii_art.AsciiArt.commit_success)

        print("=" * 80)
        print("Commit accepted, well done! This is the {}th commit!".format(
            commit_number))
        print("=" * 80)
  else:
    print(ascii_art.AsciiArt.homer_woohoo)
