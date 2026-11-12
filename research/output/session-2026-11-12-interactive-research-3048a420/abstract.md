# Quality Monitor: 1220 phantom missed steps, end-of-print stall and 0.25 mm metric

The 1220 missed steps after the HP4 logo were real stalls from a bogus "fly home" move that the RRF host simulator wrote at every standstill; fixing that (motor-position tracking plus two thread races, host build only) and regenerating the .can files gives 0. The monitor's end-of-print work also fell from up to 287 ms to 0.1 ms, an 891 mm position-error spike was fixed, and "Path within 0.25 mm" was added.

[Full report and evidence](report.md)
