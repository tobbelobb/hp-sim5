Still missing is the `--full-auto` feature, where the script looks at cost
and keeps trying until cost looks good, or prints some informative
warning about data quality if calibration does not converge.
The `--full-auto` should write values to the machine's config file
and update the running machine upon a successful calibration run.

The previous "## Troubleshooting quick tips" section was bogus.
But we need a Troubleshooting section.

The "## Subcommands, Debug Commands, and Internal Commands" section is
incomplete. It would be good to actually document the subcommands.

The "## Understanding the output" section is too superficial.
It needs to be rewritten by a human.

The "## Pointwise cost mode (default)" heading makes no sense once the
geometric cost mode is gone.
The whole section is a bit strange, it mainly explains how the outlier-
robust fitting works, and then what gets plotted in the histogram if one
chooses to plot one.


