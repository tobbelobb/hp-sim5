# Paused before final review and integration

All six frozen primary attempts are complete and pushed. Development maximum
anchor error improved 3.6959 to 2.8529 mm at 1.064117 times simulated effort.
Evaluation improved 3.4116 to 2.7518 mm at 1.070853 times effort. The nominal
candidate failed before fitting, so the 4mid force default is not adopted.

A minimal encoder reply-budget patch passed producer checks and is pushed as
95a0af8f. A valid six-second WebSocket reply failed under stock code at about
five seconds and succeeded with the patch. Firmware HTTP was stubbed; no
full-autocal run used the patch. Final separate review and integration remain
pending because the human requested pause. All raw failures are retained.

See [complete experiment report](../lower-force/report.md) and
[resume state](../CONTINUE.md). No more collection is authorized.
