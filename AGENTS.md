# js Style Guide

Don't check for null with `if (!someVar)`. Check for null with `if (someVar === null)`.

# General Coding Style

Don't sanity check all the input variables in all the functions by default.
Sanity check input variables from userland.
Handle or avoid division-by zero.

Write unit test to ensure any bugs you fix don't regress again.

Prefer small code changes and shallow stack traces.
Don't write "safe" setters/getters unless you're asked to.
