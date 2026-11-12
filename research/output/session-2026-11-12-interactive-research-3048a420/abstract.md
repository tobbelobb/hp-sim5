# Quality Monitor: 1220 phantom missed steps, end-of-print stall and 0.25 mm metric

The 1220 missed steps after the HP4 logo are real stalls caused by a bogus "fly home in 63 ms" move that the RRF host simulator writes at every standstill into 8 of 10 logo .can files; the monitor's end-of-print work dropped from up to 287 ms to 0.1 ms, an 891 mm position-error spike was fixed, and "Path within 0.25 mm" was added.

[Full report and evidence](report.md)
