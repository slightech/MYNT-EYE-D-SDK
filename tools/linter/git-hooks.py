#!/usr/bin/env python

import subprocess
import sys


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


def get_git_repo_root(some_folder_in_root_repo='./'):
    """Get the root folder of the current git repository."""
    return run_command_in_folder('git rev-parse --show-toplevel',
                                 some_folder_in_root_repo)


def get_linter_subfolder(root_repo_folder):
    """Find the subfolder where this linter is stored."""
    return run_command_in_folder("git submodule | awk '{ print $2 }'" +
                                 " | grep linter", root_repo_folder)


def main():
    # Get git root folder.
    repo_root = get_git_repo_root()

    # Get linter subfolder
    linter_subfolder = get_linter_subfolder(repo_root)

    # Append linter folder to the path so that we can import the linter module.
    linter_folder = repo_root + "/" + linter_subfolder
    sys.path.append(linter_folder)

    import linter

    linter.linter_check(repo_root, linter_subfolder)


if __name__ == "__main__":
    main()
