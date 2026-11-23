"""Group edge_replay.mjs output by unique (machine, edge ramp) and list rule changes."""
import json, collections, sys
groups = {}
for line in open(sys.argv[1]):
    r = json.loads(line)
    t = json.load(open(r['file']))
    key = (t.get('machine_type'), json.dumps(t['force_tuning']['edge_samples']))
    groups.setdefault(key, []).append(r)
count = collections.Counter()
for (machine, _), rs in sorted(groups.items(), key=lambda kv: str(kv[0][0])):
    r = rs[0]
    count[(machine, 'same' if r['same'] else 'CHANGED')] += 1
    if not r['same']:
        print('CHANGED', machine, 'old edge %.3f (%s)' % (r['old']['edge'] or 0, r['old']['reason']),
              '| new needs %d samples, logged %d' % (r['new']['n'], len(json.load(open(r['file']))['force_tuning']['edge_samples'])),
              '| e.g.', rs[0]['file'].split('hp-sim5/')[-1])
for key, value in sorted(count.items()):
    print(value, *key)
print(sum(count.values()), 'unique ramps from', sum(map(len, groups.values())), 'files')
