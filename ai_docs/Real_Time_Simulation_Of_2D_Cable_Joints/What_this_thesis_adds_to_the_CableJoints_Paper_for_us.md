<gpt-5.2>
### What CableJoints (2018) gives you (baseline)

Müller et al.’s **Cable Joints** paper is built around a few simplifying assumptions—most notably that **cables don’t slide on rotating objects** and that you simulate *the effect of the cable on the bodies* rather than the cable itself. ([matthias-research.github.io][1])
Mechanically, it models a cable as a **sequence of unilateral “upper distance” constraints** between **dynamically-updated tangent attachment points**, and it **dynamically updates rest lengths** (via stored surface-length on bodies) as bodies roll. ([matthias-research.github.io][1])

This is why it’s so fast and robust: the cost scales with “number of contacted bodies”, not “rope segments”.

---

### What the 2025 thesis adds (the useful deltas)

The BTH thesis (“**Real-time Simulation of 2D Cable Joints with Pinched and Sliding Joints using Velocity Constraints**”, June 2025) explicitly targets two gaps: **pinching** and **sliding**.

#### 1) A concrete “pinch joints” model (cable has thickness)

The thesis’ core extension is **pinch joints**: detect when a body intersects the *wrapped/stored* cable portion (which CableJoints doesn’t physically represent), then prevent “clipping through the cable” by adding constraints and/or restructuring the joint chain.

Key bits that translate *directly* to XPBD:

* It introduces an explicit **cable half-width `w`** and offsets tangent attachments by `w` (so constraints are effectively solved along the cable midline).
* For a **transitional pinch** (pinching body is adjacent in the joint chain), it “collapses” the affected segment to **zero-length** and (crucially) adds a **contact constraint** enforcing a minimum surface separation **`d ≥ 2w`** between the pinching bodies.
* For **non-transitional pinches**, it restructures the cable locally into multiple joints (so the pinching body becomes part of the joint sequence), while still needing only **one contact constraint per pinching body pair** in the incompressible-width case.

In XPBD terms, that’s basically:

* keep your usual **segment “max length” inequality** (`L ≤ l`)
* add a **distance/min-separation inequality** (`d ≥ 2w`) at pinch contacts
* plus some **topology bookkeeping** (splitting/merging joints) when a pinch begins/ends.

#### 2) A pipeline/order-of-operations that’s very ECS-friendly

The thesis spells out a full per-timestep order (their Algorithm 1), including pinch detection and configuration before constraint solve.
That’s gold for ECS because it cleanly separates into systems:

1. Update tangents + update stored cable/rest lengths (“surface distance” bookkeeping)
2. Remove/merge joints when stored cable goes negative (and handle pinch-specific merge cases)
3. Split joints if straight segments intersect new bodies
4. Detect pinch candidates (broad → narrow → confirm) and configure pinch joints
5. Sliding update
6. Build constraints (segments + contacts) and solve

#### 3) Sliding support (beyond CableJoints’ “no sliding” assumption)

CableJoints assumes no sliding (by design). ([matthias-research.github.io][1])
The thesis extends the model to support **sliding cables with friction**, citing Coulibaly’s “unified formulation” as the basis.
Even if you don’t adopt their exact friction math, the structural takeaway is: **sliding is treated as an additional update step** that can coexist with the same joint/segment representation and constraint solve ordering.

#### 4) Practical implementation gotchas (worth stealing)

A couple of “this will bite you” notes are genuinely useful in a production sim:

* They add a tiny **stored-cable buffer** to prevent floating-point noise from removing joints spuriously.
* They explicitly warn that having **two consecutive joints on the same body** breaks tangent updating (infinite tangent solutions), so pinch removal needs a merge step.

#### 5) Why it matters to your architecture choice

They note their Unity prototype had a layered-solver problem: the engine could override some cable impulses, and they suggest integrating the cable constraints “directly with an engine”.
That’s basically an argument *for* your ECS approach: one authoritative solver pass where rigid + cable constraints live together.

---

## How to apply this to an XPBD + ECS CableJoints implementation (actionable)

If you want the “highest value per LOC” additions:

### A) Add pinch without turning the cable into a particle chain

1. **Represent cable width** (`w`) and offset your tangent attachment points accordingly.
2. When a pinch is detected, generate a **PinchContact** and add an XPBD inequality constraint enforcing **`d ≥ 2w`** between the pinching bodies at closest points.
3. For transitional pinches, allow **zero-length segments** (keep the joint chain intact) and rely on the contact constraint to do the “don’t clip through cable” work.
4. Only if you need full generality, implement the **non-transitional pinch topology rewrite** (extra joints).

### B) ECS breakdown (one way to map their algorithm)

* **Entities**

  * `RigidBody`
  * `Cable` (owns a dynamic buffer/list of joint entity IDs)
  * `CableJoint` (paired body `b_i`, attachments `a_t,i` / `a_h,i`, stored cable `s_i`)
  * `CableSegment` (tail joint ref, rest length `l_i`)
  * `Constraint` entities/components (`SegmentMaxLen`, `PinchMinSep`, optional friction constraints)

* **Systems (match their timestep order)**

  1. `CableAttachmentUpdateSystem` (recompute tangents, surfaceDist bookkeeping)
  2. `CableTopologySystem` (remove/merge joints, split on intersections)
  3. `PinchDetectionSystem` (broad/narrow/confirm)
  4. `PinchConfigureSystem` (zero-length segments, joint insertion for non-transitional)
  5. `SlidingUpdateSystem` (if you want it)
  6. `ConstraintBuildSystem` then `XPBDSolveSystem`

### C) Performance note you should care about

They explicitly motivate broad-phase collision filtering (e.g., sweep-and-prune) to avoid quadratic pair checks.
In ECS, that often becomes a shared “physics broadphase” service that both rigid contacts and pinch detection query.
</gpt-5.2>


<user>
What kind of objects can potientially be "pinching" on wrapped cable in this thesis? Do they mean any rigid object can pinch a wrapped cable and create friction?

I'm very interested if any of this can generalize in such a way that spool buildup (line wrapped on top of itself in multiple layers) can be modelled within CableJoints alone. I've had a plan for a while to create a "transition window" between CableJoints and a chain of rigid bodies close to any spool, because I've thought I need rigid bodies in order to model realistic spool buildup effects. Does this thesis point to a potential way to model spool buildup more cheaply or simpler than that?
</user>

<gpt-5.2>
### What can be “pinching” in the thesis?

In *this* thesis, **pinching is not a special “spool-only” thing**. It’s defined as:

> **A body intersects the wrapped portion of cable stored in a joint link.**

So the “pinching object” can be **any rigid body** in the scene that comes into the wrapped-cable region of some other body’s joint-link. In their implementation they only test **circular bodies** (and they explicitly note polygons would create multiple possible contacts).

Mechanically, when a pinch happens between **two circles**, they treat it as producing **exactly one contact point** where the cable must keep the bodies separated.

How do they *find* candidates?

* They do a **“margined intersection”** check where each body’s border is expanded by the **full cable width** to find bodies that *might* pinch a given cable.

### Does pinching “create friction” in their sense?

Not by itself.

Pinching mainly adds a **normal separation constraint** so the cable (with thickness) can “push the bodies apart”:

* They add a **contact constraint** that enforces **minimum surface distance** `d ≥ 2w` (where `w` is half cable width).

The *friction* part lives in their **sliding extension** (Coulibaly-based), which is conceptually separate. They say they didn’t need special coupling between pinch and sliding *except* they had to hack in a **constant friction amount** for non-transitional pinch joints because the “middle joint” has ~zero wrapped cable, making their wrapped-length-based friction ineffective.

So: **pinch ≈ “thickness + contact separation”**, while **friction ≈ “sliding model”** (with one special-case constant for a pinch configuration).

---

## About spool buildup: does the thesis let you model it “inside CableJoints” cheaply?

### The bad news

The thesis’ pinch idea does **not** directly solve spool buildup, because CableJoints (and their extension) still **do not represent the wrapped cable as geometry**—it’s *just* attachment points + a float “stored cable length.”

Spool buildup is fundamentally **cable-on-itself layering**, i.e. the effective drum radius changes because the wrapped cable is **itself a body** in the contact sense. The thesis’ pinch contacts are **body–body contacts** triggered by a body intersecting the “stored cable” band on another body, not **cable–cable self contact**.

So if your goal is *full* realism (layer-to-layer contact, packing, crossovers, flange interactions, etc.), the thesis doesn’t magically remove the need for “extra degrees of freedom” near the spool.

### The good news: it *does* point to a cheaper “effective radius” model

Even though they don’t do spool buildup, the thesis gives you two ingredients that make a **cheap buildup approximation** feel more principled:

1. They introduce an explicit **cable width** via `w` and offset tangency/attachments by `w`.
2. They treat cable segments as having **height `2w`** for intersection logic.

That’s basically handing you a usable **cable diameter `d = 2w`**, which is exactly what you need to drive a “radius grows as cable winds” model.

### A simple way to do spool buildup *within CableJoints alone*

If what you mainly need is **changing mechanical advantage / payout-per-rev** (the big practical effect), you can likely avoid your “transition window into rigid-body chain” and do this:

* Make the spool/drum a **special rolling body with dynamic radius** `r_eff`.
* Let `s` be “stored cable length on that spool” (CableJoints already tracks this concept; the thesis does too).
* Compute `r_eff = r0 + f(s, d, width)` where `d = 2w`.

Two common approximations:

**A) 2D “every wrap makes a new layer” (fastest, roughest)**
Assume each full turn adds one cable diameter to radius:

* wraps `N ≈ s / (2π r_eff)` (implicit) → solve with a small iteration or use a quadratic sum-of-circumferences approximation.
* then `r_eff ≈ r0 + d * N`.

**B) Include spool width (still cheap, closer to reality)**
In reality, radius increases after ~`turns_per_layer ≈ width / d` turns. So:

* track a fractional “turn count” on the spool
* increase radius by `d` each time you complete `turns_per_layer` turns

Once you have `r_eff`, you plug it into **only two places**:

1. **Tangent point computation** (tangent to a circle of radius `r_eff`)
2. **surfaceDist / wrapped-arc length update** (arc length = `r_eff * Δθ`)

That keeps everything “CableJoints-style”: no rope particles, no rigid chain, and it directly gives you the payoff you likely care about most (radius-dependent length change and torque mapping).

### Where pinch-joints *might* still help around a spool

If you have scenarios like:

* cable being squeezed between the spool and a nearby roller/guide,
* or the cable wedged in a tight gap between two pulleys,

then pinch-joints give you a clean recipe: add a **body–body min-separation** contact `d ≥ 2w` when the “stored cable band” is being intruded.
That’s relevant to *guides* and *nearby bodies*, not to the cable’s own layering.
</gpt-5.2>
