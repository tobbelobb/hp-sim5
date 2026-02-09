
### 1) What shape is that called?

Geometrically it’s just the **union of a disk** (your original circle) and a **circular sector of a larger circle** stuck onto it — i.e. a **piecewise-circular (two-radius) profile**.

There isn’t one universally standard name, but in mechanics/design you’ll often see it described as a **two-radius cam** / **circular cam with a lobe** / **lobed wheel**, because the boundary is made from **two circular arcs with different radii** (and usually a “transition” between them).

If you literally “swap in” a larger-radius arc over some angle and keep the rest as the smaller circle, you also get **two corner points** at the joins (curvature jumps), unless you add a fillet/blend.

---

### 2) When it rolls on a flat surface, what direction is the contact/collision force?

If you mean the **normal (collision) force from the flat ground**, then for ideal rigid contact:

* The **ground’s contact normal is perpendicular to the ground** → **vertical** (in world coordinates).
* So the **normal collision impulse/force is vertical**, not “radial through your circle’s center” and not “along the tangent”.

The only time “radial through the center” happens is the special case of a perfect circle rolling: at the bottom contact point the radius happens to be vertical too.

---

### 3) “Somewhere in between” — exactly where?

There are two different “normals” people mix up:

#### A) Normal of the *ground contact* (what the ground pushes along)

For a flat ground, the **contact normal is vertical**. Full stop (for the normal component).

If you include friction, the **total contact force** can tilt away from vertical because it gains a tangential component:

* Normal component: (N) (vertical)
* Friction/tangent component: (F_t) (horizontal), with (|F_t| \le \mu N)

So the resultant force angle from vertical is
[
\beta = \arctan!\left(\frac{|F_t|}{N}\right) \le \arctan(\mu).
]

#### B) Normal of the *wheel’s boundary* (line to the local center of curvature)

For a smooth curve, the boundary normal at the contact point points toward the **local center of curvature**.
If the contacting part is a circular arc, that normal goes through **that arc’s center**.

But **for rolling contact on a plane**, the boundary tangent at the contact must be horizontal, so the boundary normal is vertical there anyway (if it’s a smooth single contact point). The *interesting part* is: **that vertical line generally does NOT pass through your original circle’s center**.

---

### 4) Angle relative to the circle center (what you were probably visualizing)

Let:

* Ground is the line (y=0).
* Contact point is (P=(x_p,,0)).
* Your chosen reference center (e.g. the original circle’s center) is (O=(x_0,,y_0)) in world coordinates.

Normal force direction is vertical: (\hat n=(0,1)).

The radius-to-contact vector is ( \vec r = P - O = (x_p-x_0,,-y_0)).

The angle (\gamma) between the **upward contact force** and the **(inward) radial line from contact to center** is:
[
\gamma = \arctan!\left(\frac{|x_p-x_0|}{y_0}\right).
]

* If (x_p=x_0), then (\gamma=0): force is “radial” (circle case).
* Otherwise it’s **between radial and tangential** (your intuition), but the “exactly where” is determined by the **horizontal offset** between the center and the contact point, divided by the **center height** above the ground.

That offset also means the normal force creates a torque about (O):
[
\tau = N ,(x_p-x_0)
]
(sign depending on side).

---

### 5) One important edge case: the join points

If your profile has a **sharp corner** where the small-radius arc meets the big-radius arc, the “normal” is **not unique** at that exact point (you have a cone of possible supporting normals). In practice:

* contact will typically occur **slightly before/after** the corner on one of the arcs, or
* you add a **blend/fillet** so the normal changes continuously.

