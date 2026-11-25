# COPacabana PlaneA replay

On the first 13.9 kg standard-control cycle, the simulator settles within 2.5 mm of the controller's commanded pose. The real platform misses that pose by up to 20.6 mm, because in every PlaneA experiment its Y travel is 3.5–4.6 % shorter than commanded. That gap is the machine's own geometry error relative to `COPacabana.xml`, not cable physics, and it likely also drives the 176 N RMS force disagreement. [Report](report.md)
