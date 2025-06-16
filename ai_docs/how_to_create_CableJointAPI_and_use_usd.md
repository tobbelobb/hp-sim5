## Opening and Traversing USD Files with Python API

To read a USD scene, use the **USD Python API** to open the stage and then navigate its prims. For example, given a USD file (in **USDA** text form) like below:

```usda
#usda 1.0
def Xform "hello" {
    def Sphere "world" {
        float3[] extent = [(-2,-2,-2), (2,2,2)]
        color3f[] primvars:displayColor = [(0,0,1)]
        double radius = 2
    }
}
```

You can open this file and inspect its contents in Python as follows:

```python
from pxr import Usd
stage = Usd.Stage.Open("HelloWorld.usda")                     # Open the stage from file:contentReference[oaicite:0]{index=0}
xform = stage.GetPrimAtPath("/hello")                        # Get prim by path
sphere = stage.GetPrimAtPath("/hello/world")                 # Get child prim
print(xform.GetPropertyNames())                              # List properties on Xform prim:contentReference[oaicite:1]{index=1}
print(sphere.GetPropertyNames())                             # List properties on Sphere prim:contentReference[oaicite:2]{index=2}
# Access and modify an attribute on the sphere:
radius_attr = sphere.GetAttribute("radius")
print(radius_attr.Get())    # e.g. prints 2.0 (initial radius):contentReference[oaicite:3]{index=3}
radius_attr.Set(1.0)        # Modify the sphere's radius to 1.0:contentReference[oaicite:4]{index=4}
print(radius_attr.Get())    # Now prints 1.0
```

This code opens the USD stage, retrieves prims by their paths, and uses `GetAttribute().Get()` and `Set()` to read and write attribute values. To iterate through **all prims** in the stage, you can traverse it:

```python
for prim in stage.Traverse():
    print(prim.GetPath())
```

The `UsdStage.Traverse()` generator yields each prim in the stage in depth-first order. If you need to traverse from a specific prim downward, you can use `Usd.PrimRange` on that prim (e.g. `for prim in Usd.PrimRange(subprim): ...`). Using these APIs, you can programmatically explore the scene graph, query attributes, and modify them as needed.

**Sources:** USD **OpenUSD** documentation and NVIDIA Isaac Sim 4.5.0 snippets.

## Creating and Structuring USD Scenes with Prims

USD scenes are built from prims such as **Xform** (a generic transform/group), geometry (e.g. Sphere, Cube, Capsule), and special prims for physics, lights, etc. You can create a new stage and add prims in Python, or author them in USDA text. For example, the snippet below creates a simple scene with a world Xform, a Sphere, a Capsule, and a physics scene using the USD Python API:

```python
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
stage = Usd.Stage.CreateNew("Scene.usda")
root = UsdGeom.Xform.Define(stage, "/World")                         # Root Xform prim:contentReference[oaicite:12]{index=12}
sphere = UsdGeom.Sphere.Define(stage, "/World/Sphere")               # Add a Sphere geometry:contentReference[oaicite:13]{index=13}
capsule = UsdGeom.Capsule.Define(stage, Sdf.Path("/World/Capsule"))  # Add a Capsule geometry
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))  # Add a PhysicsScene prim:contentReference[oaicite:14]{index=14}
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1.0))           # Set gravity direction (Z down):contentReference[oaicite:15]{index=15}
scene.CreateGravityMagnitudeAttr().Set(981.0)                        # Set gravity magnitude (cm/s^2):contentReference[oaicite:16]{index=16}
stage.GetRootLayer().Save()  # save the stage to USDA
```

This Python code uses schema-specific classes: **UsdGeom.Xform**, **UsdGeom.Sphere**, **UsdGeom.Capsule**, etc., to **Define** new prims on the stage (creating them if they don’t exist). It also defines a **UsdPhysics.Scene** (the physics scene), and sets its gravity vector. The resulting **USDA** scene might look like:

```usda
#usda 1.0
def Xform "World" {
    def Sphere "Sphere" {
        double radius = 50
        ...                         # (other Sphere attributes like extent, displayColor)
    }
    def Capsule "Capsule" {
        double height = 100
        double radius = 25
        token axis = "Z"
        ...                         # (Capsule has radius, height, axis by default)
    }
    def PhysicsScene "physicsScene" {
        uniform token physics:gravityDirection = (0, 0, -1)    # Gravity direction:contentReference[oaicite:18]{index=18}
        uniform float physics:gravityMagnitude = 981.0         # Gravity magnitude:contentReference[oaicite:19]{index=19}
    }
}
```

In the USDA above, the **Xform** named "World" contains a **Sphere** and a **Capsule** prim. The **PhysicsScene** prim is defined with its gravity attributes (note the `physics:` namespace on gravity attributes). Standard geometry prim types like *Sphere* and *Capsule* come from the **UsdGeom** schema and have predefined attributes (e.g. `radius`, `height`, etc.). The PhysicsScene is from the **UsdPhysics** schema (part of OpenUSD’s physics schema), providing scene-wide physics properties like gravity.

**Sources:** Pixar’s OpenUSD *Hello World* tutorial (creating Xform/Sphere) and NVIDIA’s Isaac Sim 4.5.0 physics snippet.

## Using NVIDIA PhysX Extensions (PhysX APIs in USD)

NVIDIA’s Isaac Sim extends USD with PhysX physics features via **applied API schemas**. Key PhysX-related schemas include:

* **PhysxSceneAPI** – extension for PhysX-specific scene settings (on a PhysicsScene prim),
* **PhysicsRigidBodyAPI** – marks a prim as a dynamic rigid body,
* **PhysicsCollisionAPI** – marks a prim as a collideable shape,
* **PhysicsRevoluteJoint** – a prim representing a revolute (hinge) joint connecting two bodies.

**Python API Usage:** In Python, you **apply** these APIs to prims (for API schemas) or **create** the joint prims, then set their properties. For example:

```python
from pxr import PhysxSchema, UsdPhysics

# Assume stage, and a PhysicsScene "/World/physicsScene" and a prim "/World/Cube" exist
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene"))        # Enable PhysX on the scene:contentReference[oaicite:26]{index=26}
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)           # Enable continuous collision detection:contentReference[oaicite:27]{index=27}
physxSceneAPI.CreateEnableStabilizationAttr(True) # Enable PhysX stabilization:contentReference[oaicite:28]{index=28}
physxSceneAPI.CreateSolverTypeAttr("TGS")         # Set solver type to TGS:contentReference[oaicite:29]{index=29}

cubePrim = stage.GetPrimAtPath("/World/Cube")
UsdPhysics.CollisionAPI.Apply(cubePrim)           # Make the prim a collision shape:contentReference[oaicite:30]{index=30}
UsdPhysics.RigidBodyAPI.Apply(cubePrim)           # Make the prim a rigid body (dynamic):contentReference[oaicite:31]{index=31}

# Create a revolute joint connecting two bodies (body0 and body1 are UsdPrim objects for the bodies)
joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/RevoluteJoint")             # Define a RevoluteJoint prim
joint.GetBody0Rel().AddTarget(body0.GetPath())   # Attach first body:contentReference[oaicite:32]{index=32}
joint.GetBody1Rel().AddTarget(body1.GetPath())   # Attach second body:contentReference[oaicite:33]{index=33}
joint.CreateAxisAttr().Set(UsdPhysics.Tokens.X)  # Set joint axis to X-axis:contentReference[oaicite:34]{index=34}
joint.CreateLowerLimitAttr().Set(0.0)            # Set lower angle limit (degrees):contentReference[oaicite:35]{index=35}
joint.CreateUpperLimitAttr().Set(90.0)           # Set upper angle limit (degrees):contentReference[oaicite:36]{index=36}
```

In the code above, **PhysxSceneAPI** is applied to the `/World/physicsScene` prim to turn it into a PhysX simulation scene. We then set attributes like *enableCCD*, *enableStabilization*, *solverType*, etc., via the API (these calls create attributes such as `physxScene:enableCCD = True`). Next, we apply **UsdPhysics.CollisionAPI** and **UsdPhysics.RigidBodyAPI** to a prim (e.g. a Cube) to make it a dynamic rigid body with collision enabled. Finally, we create a **PhysicsRevoluteJoint** prim and link two bodies via its `physics:body0` and `physics:body1` relationships. We set the joint’s axis and limits; for a revolute joint, USD uses a uniform token attribute `physics:axis` (allowed values "X","Y","Z") and float attributes `physics:lowerLimit` and `physics:upperLimit` (in degrees) to define its motion range.

**USDA Representation:** After applying these, the USD stage (in text form) will reflect the applied schemas and attributes. For example, the PhysicsScene prim will list **PhysxSceneAPI** in its `apiSchemas` and have PhysX attributes, and the cube prim will list CollisionAPI and RigidBodyAPI:

```usda
def PhysicsScene "physicsScene" (
    apiSchemas = ["PhysxSceneAPI"]
) {
    bool physxScene:enableCCD = 1                    # PhysX scene settings:contentReference[oaicite:43]{index=43}
    bool physxScene:enableStabilization = 1
    token physxScene:solverType = "TGS"
    float physics:gravityMagnitude = 981.0
    float3 physics:gravityDirection = (0, 0, -1)
}
def Cube "Cube" (
    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"]
) {
    double size = 50
    # (Cube geometry attributes, e.g., size, omitted for brevity)
}
def PhysicsRevoluteJoint "RevoluteJoint" {
    rel physics:body0 = </World/BodyA>               # Joint connected bodies:contentReference[oaicite:44]{index=44}
    rel physics:body1 = </World/BodyB】
    uniform token physics:axis = "X"                 # Revolute axis:contentReference[oaicite:45]{index=45}
    float physics:lowerLimit = 0                     # Lower angle limit:contentReference[oaicite:46]{index=46}
    float physics:upperLimit = 90                    # Upper angle limit:contentReference[oaicite:47]{index=47}
}
```

In the USDA above, note how applied API schemas appear: the Cube prim’s metadata `apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"]` indicates it has those APIs applied. Similarly, the PhysicsScene has `PhysxSceneAPI` applied, and the joint prim has its properties defined (the relationships `physics:body0` and `physics:body1`, and the axis/limit attributes under the **UsdPhysics.RevoluteJoint** schema).

**Sources:** Isaac Sim 4.5 documentation and examples, OpenUSD Physics schema reference and USD API reference for RevoluteJoint.

## Defining and Using Custom API Schemas (e.g. *CableJointAPI*)

USD is extensible – you can create **custom schema classes** to attach domain-specific data to prims. A **“codeless” API schema** can be defined entirely in a plugin’s `schema.usda` (no C++ code), or you can use the `usdGenSchema` tool to generate C++/Python classes for your schema. In both cases, the schema defines new properties and can be **applied** to prims just like USD’s built-in API schemas.

For example, imagine a custom *CableJointAPI* to tag prims with properties of a hypothetical cable joint. You would define a class in a `schema.usda` file, something like:

```usda
class "CableJointAPI" (
    inherits = </APISchemaBase>,
    customData = {
        token apiSchemaType = "singleApply"
    }
)
{
    float cableJoint:restLength (
        doc = "Rest length of the cable"
    )
    double3 cableJoint:attachPointA (
        doc = "Attachment point on body A"
    )
    double3 cableJoint:attachPointB (
        doc = "Attachment point on body B"
    )
}
```

This declares *CableJointAPI* as a single-apply API schema (deriving from APISchemaBase) with three example properties (`restLength`, `attachPointA/B`). With a codeless plugin, simply registering this schema makes it available at runtime. With code generation, running `usdGenSchema` would produce C++/Python classes for `CableJointAPI` as well.

**Using the Custom API in USD:** Once your schema is registered, you can apply it to prims and author its properties. For example, in **USDA** you would apply it via the `apiSchemas` metadata and set its attributes under the appropriate namespace:

```usda
def Xform "SomeLink" (
    prepend apiSchemas = ["CableJointAPI"]
) {
    custom float cableJoint:restLength = 5.0
    custom double3 cableJoint:attachPointA = (0, 0, 0)
    custom double3 cableJoint:attachPointB = (1, 0, 0)
}
```

Here, the prim `/SomeLink` has `CableJointAPI` applied (listed in `apiSchemas`), and the custom attributes (restLength, attachPointA/B) are authored on it. These `custom` qualifiers indicate they aren’t part of USD’s core schemas, but defined by our plugin.

**Using the Custom API in Python:** If you generated Python classes, you can import and use them. For example, if the schema plugin library is named `UsdSchemaExamples` (as in USD’s tutorial), you would do:

```python
from pxr import Usd, UsdSchemaExamples
prim = stage.GetPrimAtPath("/SomeLink")
# Apply the API (for single-apply schemas, an Apply() method is generated)
UsdSchemaExamples.CableJointAPI.Apply(prim)
# Now set or get attributes via the schema class:
cj = UsdSchemaExamples.CableJointAPI(prim)
cj.GetRestLengthAttr().Set(5.0)
print(cj.GetRestLengthAttr().Get())  # prints 5.0
```

If using a codeless schema (no generated code), you won’t have a dedicated Python class, but you can still author the attributes using the generic `prim.GetAttribute("cableJoint:restLength").Set(5.0)` approach. The key point is that the prim’s `apiSchemas` metadata will reflect that *CableJointAPI* is applied, which **USD’s type system** uses to know the prim has those properties.

**Example:** USD’s tutorial defines an API schema `ParamsAPI` with some custom attributes. In the **USDA** below, you can see it applied to a prim and the attributes set:

```usda
def Xform "Object" (
    prepend apiSchemas = ["ParamsAPI"]
) {
    custom double params:mass = 1.0
    custom double params:velocity = 10.0
    custom double params:volume = 4.0
}
```

In Python, you can apply and use that API as follows:

```python
from pxr import UsdSchemaExamples  # assume the plugin is loaded
obj = stage.GetPrimAtPath("/Object")
UsdSchemaExamples.ParamsAPI.Apply(obj)                        # Apply the custom API schema:contentReference[oaicite:55]{index=55}
params = UsdSchemaExamples.ParamsAPI(obj)
print("mass:", params.GetMassAttr().Get())                    # Read the custom attributes:contentReference[oaicite:56]{index=56}
params.GetMassAttr().Set(2.0)                                 # Modify attributes via generated API
```

This demonstrates the workflow: define the schema, apply it to prims (recorded in `apiSchemas`), and use either the generated API class or standard USD attribute calls to manipulate the custom data. By using **codeless schemas**, you can even skip generating code – just define in `.usda` and the data can be authored and read – though generated classes make it more convenient (providing `Apply()` and typed Get/Set methods).

In summary, custom API schemas let you **extend USD** in a structured way. They can be distributed as plugins (with `schema.usda` and a `plugInfo.json` entry) and used in any USD-compatible application, including Isaac Sim. Whether codeless or code-generated, their usage in Python and USDA is similar to built-in schemas: apply the API to a prim, then read/write its attributes via USD’s API or the schema’s convenience methods.

**Sources:** Pixar’s USD documentation on schema generation and NVIDIA/OpenUSD examples of using an applied API schema in Python.
