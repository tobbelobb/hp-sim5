#!/bin/sh
# rrg.sh - Searches for class/struct definitions in two repositories with optional custom formatting.

# --- 1. Argument Handling and Flag Check ---

USE_GERP=0

# Check if the first argument is the custom flag
if [ "$1" = "--gerp" ]; then
    USE_GERP=1
    shift # Removes the first argument (--gerp), moving the class name to $1
fi

# Check for the class name argument
if [ -z "$1" ]; then
    echo "Usage: $0 [--gerp] <ClassNameOrStructName>"
    echo "  --gerp: Use custom 'vim ;#' output format (for clickable terminal links)."
    exit 1
fi

CLASS_NAME="$1"
# The regular expression to search for (using the shifted $1/CLASS_NAME)
REGEX='^\s*(class|struct|enum class|float|size_t)\s+'"$CLASS_NAME"'\b'


# --- 2. Conditional Execution (Repo 1: ../ReprapFirmware) ---

if [ "$USE_GERP" -eq 1 ]; then
    # Custom 'git-gerp' behavior with colors and vim prefix
    # 1. Run git grep with colors
    # 2. Add the repository path prefix (../ReprapFirmware/)
    # 3. Apply the custom sed to format output for vim-jump
    git -C ../ReprapFirmware -c color.grep.separator=cyan -c color.grep.linenumber=green grep -n --color=always -E "$REGEX" -- src |
        sed -E 's|^|../ReprapFirmware/|' |
        sed -E 's/(.*\[36m)([:-])(\[m\[32m[0-9]+\[m\[36m)\2/vim \1 +\3 ;#/g'

else
    # Standard 'git grep' behavior with simple prefix
    # 1. Run standard git grep
    # 2. Add the repository path prefix (../ReprapFirmware/)
    git -C ../ReprapFirmware grep -n --full-name -E "$REGEX" -- src |
        sed 's|^|../ReprapFirmware/|'
fi

# --- 3. Conditional Execution (Repo 2: Current Repo) ---

if [ "$USE_GERP" -eq 1 ]; then
    # Custom 'git-gerp' behavior with colors and vim prefix
    # 1. Run git grep with colors
    # 2. Apply the custom sed to format output for vim-jump
    git -C . -c color.grep.separator=cyan -c color.grep.linenumber=green grep -n --color=always -E "$REGEX" -- include src |
        sed -E 's/(.*\[36m)([:-])(\[m\[32m[0-9]+\[m\[36m)\2/vim \1 +\3 ;#/g'
else
    # Standard 'git grep' behavior
    # 1. Run standard git grep
    git -C . grep -n --full-name -E "$REGEX" -- include src
fi
