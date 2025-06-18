# USD Terms and Concepts [](https://openusd.org/release/glossary.html\#usd-terms-and-concepts "Permalink to this heading")

USD introduces quite a few terms and concepts, some of which, unavoidably,
already have different meanings in other contexts, so it can be quite daunting
to make sense of all of our documentation and code until you have become fully
indoctrinated. This document attempts to define and explain all the major
concepts and behaviors referred to in other parts of the USD documentation.

- [Active / Inactive](https://openusd.org/release/glossary.html#active-inactive)

- [API Schema](https://openusd.org/release/glossary.html#api-schema)

- [Assembly](https://openusd.org/release/glossary.html#assembly)

- [Asset](https://openusd.org/release/glossary.html#asset)

- [AssetInfo](https://openusd.org/release/glossary.html#usdglossary-assetinfo)

- [Asset Resolution](https://openusd.org/release/glossary.html#usdglossary-assetresolution)

- [Attribute](https://openusd.org/release/glossary.html#attribute)

- [Attribute Block](https://openusd.org/release/glossary.html#attribute-block)

- [Attribute Connection](https://openusd.org/release/glossary.html#attribute-connection)

- [Attribute Variability](https://openusd.org/release/glossary.html#attribute-variability)

- [Change Processing](https://openusd.org/release/glossary.html#change-processing)

- [Class](https://openusd.org/release/glossary.html#class)

- [Clips](https://openusd.org/release/glossary.html#usdglossary-clips)

- [Collection](https://openusd.org/release/glossary.html#collection)

- [Component](https://openusd.org/release/glossary.html#component)

- [Composition](https://openusd.org/release/glossary.html#composition)

- [Composition Arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs)

- [Connection](https://openusd.org/release/glossary.html#usdglossary-connection)

- [Crate File Format](https://openusd.org/release/glossary.html#crate-file-format)

- [Def](https://openusd.org/release/glossary.html#def)

- [Default Value](https://openusd.org/release/glossary.html#usdglossary-defaultvalue)

- [Direct Opinion](https://openusd.org/release/glossary.html#direct-opinion)

- [EditTarget](https://openusd.org/release/glossary.html#edittarget)

- [Fallback](https://openusd.org/release/glossary.html#fallback)

- [Flatten](https://openusd.org/release/glossary.html#flatten)

- [Gprim](https://openusd.org/release/glossary.html#usdglossary-gprim)

- [Group](https://openusd.org/release/glossary.html#usdglossary-group)

- [Hydra](https://openusd.org/release/glossary.html#hydra)

- [Index](https://openusd.org/release/glossary.html#usdglossary-index)

- [Inherits](https://openusd.org/release/glossary.html#usdglossary-inherits)

- [Instanceable](https://openusd.org/release/glossary.html#instanceable)

- [Instancing](https://openusd.org/release/glossary.html#usdglossary-instancing)

- [Interpolation](https://openusd.org/release/glossary.html#interpolation)

- [IsA Schema](https://openusd.org/release/glossary.html#isa-schema)

- [Kind](https://openusd.org/release/glossary.html#usdglossary-kind)

- [Layer](https://openusd.org/release/glossary.html#usdglossary-layer)

- [Layer Offset](https://openusd.org/release/glossary.html#layer-offset)

- [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack)

- [List Editing](https://openusd.org/release/glossary.html#list-editing)

- [LIVERPS Strength Ordering](https://openusd.org/release/glossary.html#liverps-strength-ordering)

- [Load / Unload](https://openusd.org/release/glossary.html#load-unload)

- [Localize](https://openusd.org/release/glossary.html#localize)

- [Metadata](https://openusd.org/release/glossary.html#usdglossary-metadata)

- [Model](https://openusd.org/release/glossary.html#usdglossary-model)

- [Model Hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy)

- [Namespace](https://openusd.org/release/glossary.html#usdglossary-namespace)

- [Opinions](https://openusd.org/release/glossary.html#opinions)

- [Over](https://openusd.org/release/glossary.html#usdglossary-over)

- [Path](https://openusd.org/release/glossary.html#usdglossary-path)

- [Path Translation](https://openusd.org/release/glossary.html#usdglossary-pathtranslation)

- [Payload](https://openusd.org/release/glossary.html#usdglossary-payload)

- [Prim](https://openusd.org/release/glossary.html#usdglossary-prim)

- [Prim Definition](https://openusd.org/release/glossary.html#usdglossary-primdefinition)

- [PrimSpec](https://openusd.org/release/glossary.html#usdglossary-primspec)

- [PrimStack](https://openusd.org/release/glossary.html#primstack)

- [Primvar](https://openusd.org/release/glossary.html#usdglossary-primvar)

- [Property](https://openusd.org/release/glossary.html#usdglossary-property)

- [PropertySpec](https://openusd.org/release/glossary.html#propertyspec)

- [PropertyStack](https://openusd.org/release/glossary.html#propertystack)

- [Proxy](https://openusd.org/release/glossary.html#proxy)

- [PseudoRoot](https://openusd.org/release/glossary.html#usdglossary-pseudoroot)

- [Purpose](https://openusd.org/release/glossary.html#usdglossary-purpose)

- [References](https://openusd.org/release/glossary.html#usdglossary-references)

- [Relationship](https://openusd.org/release/glossary.html#usdglossary-relationship)

- [Relocates](https://openusd.org/release/glossary.html#relocates)

- [Root LayerStack](https://openusd.org/release/glossary.html#usdglossary-rootlayerstack)

- [Schema](https://openusd.org/release/glossary.html#usdglossary-schema)

- [Session Layer](https://openusd.org/release/glossary.html#usdglossary-sessionlayer)

- [Specializes](https://openusd.org/release/glossary.html#usdglossary-specializes)

- [Specifier](https://openusd.org/release/glossary.html#usdglossary-specifier)

- [Stage](https://openusd.org/release/glossary.html#usdglossary-stage)

- [Stage Traversal](https://openusd.org/release/glossary.html#stage-traversal)

- [Subcomponent](https://openusd.org/release/glossary.html#usdglossary-subcomponent)

- [SubLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers)

- [TimeCode](https://openusd.org/release/glossary.html#usdglossary-timecode)

- [TimeCodes Scaled to Real Time](https://openusd.org/release/glossary.html#timecodes-scaled-to-real-time)

- [TimeSample](https://openusd.org/release/glossary.html#usdglossary-timesample)

- [Typed Schema](https://openusd.org/release/glossary.html#typed-schema)

- [User Properties](https://openusd.org/release/glossary.html#user-properties)

- [Value Clips](https://openusd.org/release/glossary.html#usdglossary-valueclips)

- [Value Resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution)

- [Variability](https://openusd.org/release/glossary.html#usdglossary-variability)

- [Variant](https://openusd.org/release/glossary.html#usdglossary-variant)

- [VariantSet](https://openusd.org/release/glossary.html#usdglossary-variantset)

- [Visibility](https://openusd.org/release/glossary.html#usdglossary-visibility)


* * *

## [Active / Inactive](https://openusd.org/release/glossary.html\#id365) [](https://openusd.org/release/glossary.html\#active-inactive "Permalink to this heading")

_Activation_ is a [metadatum](https://openusd.org/release/glossary.html#usdglossary-metadata)/behavior of [prims](https://openusd.org/release/glossary.html#usdglossary-prim) that models a “non destructive and reversible prim
deletion” from a [stage](https://openusd.org/release/glossary.html#usdglossary-stage) . Prims are active by default,
which means they and their active children will be [composed](https://openusd.org/release/glossary.html#usdglossary-composition) and visited by [stage traversals](https://openusd.org/release/glossary.html#usdglossary-stagetraversal). However, by making a prim _inactive_ by calling
[UsdPrim::SetActive(false)](https://openusd.org/release/api/class_usd_prim.html#ac156eed30c42c013c4a4debf580ce17f),
we prevent the prim itself from being visited by default traversals, and we
also prevent the prim’s descendant prims from even being composed on the stage,
which makes _deactivation_ a useful tool for pruning unneeded scene description
for scalability and complexity management.

In the following example, the prim at [path](https://openusd.org/release/glossary.html#usdglossary-path)/Parent has been deactivated, so /Parent/Child1 and other
descendants will not be composed. However, /Parent’s active
metadatum can be overridden to _true_ in a stronger layer, which would cause
/Parent/Child1 and etc. to compose onto the stage.

Example of a deactivated prim [](https://openusd.org/release/glossary.html#id311 "Permalink to this code")

```
def Xform "Parent" (
    active = false
)
{
    def Mesh "Child1"
    {
    }
    # other siblings of "Child1" ...
}

```

## [API Schema](https://openusd.org/release/glossary.html\#id366) [](https://openusd.org/release/glossary.html\#api-schema "Permalink to this heading")

An API schema is a prim [Schema](https://openusd.org/release/glossary.html#usdglossary-schema) that serves as an
interface or _API_ for authoring and extracting a set of related data, and _may_
also contribute to a [prim’s definition](https://openusd.org/release/glossary.html#usdglossary-primdefinition). In
terms of the USD object model, an API schema is one that derives from
[UsdAPISchemaBase](https://openusd.org/release/api/class_usd_a_p_i_schema_base.html), but **not** from its subclass [UsdTyped](https://openusd.org/release/api/class_usd_typed.html).
That means [UsdPrim::IsA<UsdModelAPI>()](https://openusd.org/release/api/class_usd_prim.html#a6cc1b146723c266ae69fb873bcb2df67) will never
return true. In contrast to typed, “is a” schemas, think of API schemas as “has
a” schemas. There are three types of API schema: **non-applied, single-apply,**
**and multiple-apply.**

- **Non-applied API Schemas**

Non-applied API schemas are, in some sense, the “weakest” API schemas, since
**all** they provide is API for setting and fetching some related properties
and/or metadata, and do not contribute to a prim’s type or definition in any
way. The [UsdModelAPI](https://openusd.org/release/api/class_usd_model_a_p_i.html) is an example of a non-applied schema whose
purpose is to interact with a few pieces of prim metadata related to models
and assets; there is no way to tell whether a [UsdModelAPI](https://openusd.org/release/api/class_usd_model_a_p_i.html) is
_present_ on a prim - instead one simply uses its methods to interrogate
particular aspects.





```
Usd.ModelAPI(prim).SetKind(Kind.Tokens.subcomponent)

```





One might also create a non-applied schema to interact with some related (for
your purposes) properties that exist on some number of other typed schemas,
which may allow for more concise coding patterns.

- **Single and Multiple Apply Schemas**

Applied schemas get recorded onto a prim in its apiSchemas metadata,
which allows them to contribute to the prim’s definition, adding builtin
properties, and submitting to presence queries, using
UsdPrim::HasAPI<UsdShadeMaterialBindingAPI>(). The most common type of
API schema is **single-apply,** which should be applied before using the
schema’s authoring APIs. That is, whereas when we are authoring _typed_ schemas
we usually **assign the type** as we define the prim, before using the schema to
author data, like:





```
mesh = UsdGeom.Mesh.Define(stage, path)
mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.bilinear)

```





for applied API schemas we instead **apply the type to an existing prim** before
using the schema to author data, like:





```
bindingAPI = UsdShade.MaterialBindingAPI.Apply(prim)
bindingAPI.Bind(materialPrim)

```





**Multiple-apply schemas** can be applied to a prim more than once, requiring an
“instance name” to distinguish one application from another - the instance name
will form part of the namespace in which the schema’s properties will be
authored. [UsdCollectionAPI](https://openusd.org/release/api/class_usd_collection_a_p_i.html) is an example of a multiple-apply schema,
which [UsdShadeMaterialBindingAPI](https://openusd.org/release/api/class_usd_shade_material_binding_a_p_i.html) uses to assign materials to
_collections_ of prims, allowing multiple collections to be located on a
single prim.


Choose to create an applied API Schema when you have a group of related
properties, metadata, and possibly associated behaviors that may need to be
applied to **multiple different types of prims** . For example, if your
pipeline has a set of three attributes that get authored onto every [gprim](https://openusd.org/release/glossary.html#usdglossary-gprim), and you want to have a robust schema for authoring and
extracting those attributes, you could create an applied API schema for
them. Why not instead subclass the [typed](https://openusd.org/release/glossary.html#usdglossary-isaschema) [UsdGeomGprim](https://openusd.org/release/api/class_usd_geom_gprim.html) schema and add the attributes there? Because you would
then need to redefine all of the schema classes that derive from Gprim, thus
preventing you from taking advantage of built-in DCC support for the
[UsdGeomGprim](https://openusd.org/release/api/class_usd_geom_gprim.html)-derived classes. API schemas provide for “mix in” data
organization.

API schemas can be generated using the [USD schema generation tools](https://openusd.org/release/api/_usd__page__generating_schemas.html), and extended with custom methods,
just as typed schemas can. We use schema generation even for non-applied API
schemas that have no builtin properties, to ensure consistency.

## [Assembly](https://openusd.org/release/glossary.html\#id367) [](https://openusd.org/release/glossary.html\#assembly "Permalink to this heading")

In USD an assembly is a [kind](https://openusd.org/release/glossary.html#usdglossary-kind) of [Model](https://openusd.org/release/glossary.html#usdglossary-model). Assemblies are [group](https://openusd.org/release/glossary.html#usdglossary-group) models,
i.e. models that aggregate other models into meaningful collections. An assembly
may consist primarily of references to other models, and themselves be published
assets.

An “assembly” asset, declared in scene description [](https://openusd.org/release/glossary.html#id312 "Permalink to this code")

```
def Xform "Forest_set" (
    kind = "assembly"
)
{
    # Possibly deep namespace hierarchy of prims, with references to other assets
}

```

**See also:** [model hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy)

## [Asset](https://openusd.org/release/glossary.html\#id368) [](https://openusd.org/release/glossary.html\#asset "Permalink to this heading")

_Asset_ is a fairly common organizational concept in content-producing
pipelines. In the most generic terms in USD, an asset is something that can be
identified and located (via [asset resolution](https://openusd.org/release/glossary.html#usdglossary-assetresolution))
with a string identifier. To facilitate operations such as asset dependency
analysis, USD defines a specialized string type, [asset](https://openusd.org/release/api/_usd__page__datatypes.html), so that
all metadata and attributes that refer to assets can be quickly and robustly
identified. In content pipelines, assets are sometimes a single file (e.g. a
UV texture), or a collection of files anchored by a single file that in turn
references others. A non-defining, but important quality of assets is that
they are generally published and version-controlled. As an aid to asset
management and analysis of composed scenes, USD provides an [AssetInfo](https://openusd.org/release/glossary.html#usdglossary-assetinfo) schema. The text .usda format
uses a special syntax for asset-valued strings to make them easily
differentiable from ordinary strings, using the “@” symbol instead of quotes to
delimit their values, or “@@@” as delimiter if the asset path itself contains
the “@” character. If an asset path must contain the string “@@@” then it
should be singly escaped, as “\\@@@”. See [AssetInfo](https://openusd.org/release/glossary.html#usdglossary-assetinfo) for examples of both forms.

## [AssetInfo](https://openusd.org/release/glossary.html\#id369) [](https://openusd.org/release/glossary.html\#usdglossary-assetinfo "Permalink to this heading")

_AssetInfo_ is a metadata dictionary that can be applied to any [UsdPrim](https://openusd.org/release/glossary.html#usdglossary-prim) or [UsdProperty](https://openusd.org/release/glossary.html#usdglossary-property) to convey
asset-identification-and-management-related information. Typically a [UsdStage](https://openusd.org/release/glossary.html#usdglossary-stage) pulls in many assets through [composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs). When interacting with prims and properties
on the stage, however, the presence and location of the arcs and the identity
of the assets they target is, by design, fairly deeply hidden as an
implementation detail. We do provide low-level tools for examining the
arc-by-arc [index](https://openusd.org/release/glossary.html#usdglossary-index) for each prim, but it can be difficult
to reconstruct intent and asset identity from the arc structure
alone. **AssetInfo** provides a mechanism for identifying and locating assets
that survives composition (and even [stage flattening](https://openusd.org/release/glossary.html#usdglossary-flatten))
to allow clients to discover the [namespace location](https://openusd.org/release/glossary.html#usdglossary-namespace)
at which an asset is introduced, and generally how to construct a reference to
the asset.

Note

assetInfo authored in USD files is advisory data one supplies for
client applications to use. **It is not consulted or consumed by the**
**USD core in any way during Stage loading/composition.**

The [AssetInfo dictionary](https://openusd.org/release/api/class_usd_object.html#a568ab5eceb43fd673cad556f30df6619) can contain any
fields a client or pipeline finds useful, but promotes four core fields, which
each have direct API:

> - **identifier** (asset)
>
> Provides the asset identifier one would use to target the asset with a
> composition arc.
>
> - **name** (string)
>
> Provides the name of the asset as it would be identified, for example, in
> an asset database query.
>
> - **payloadAssetDependencies** (asset\[\])
>
> Provides what can be a substantial optimization for dynamic asset
> dependency analysis. If your asset build/publish process can pre-compute,
> at publish-time, the external asset dependencies contained inside the
> asset’s payload(s), then shot/render-time dependency analysis (for asset
> isolation) need not load the payload.
>
> - **version** (string)
>
> Provides the version of the asset that was/would-be targeted. In some
> pipelines, it may make sense to inject an asset’s revision control version
> into the published asset itself. In other pipelines, version is tracked in
> an external database, so the version assetInfo must be added in the
> referencing context when we add a reference to an asset.

A continuation of the example above that illustrates [assemblies](https://openusd.org/release/glossary.html#usdglossary-assembly):

AssetInfo on a published assembly asset [](https://openusd.org/release/glossary.html#id313 "Permalink to this code")

```
def Xform "Forest_set" (
    assetInfo = {
        asset identifier = @Forest_set/usd/Forest_set.usd@
        string name = "Forest_set"
    }
    kind = "assembly"
)
{
    # Example of an asset that embeds the '@', and so must be delimited
    # with the "@@@" form
    asset primvars:texture = @@@body_decal.exr@v3@@@
}

```

## [Asset Resolution](https://openusd.org/release/glossary.html\#id370) [](https://openusd.org/release/glossary.html\#usdglossary-assetresolution "Permalink to this heading")

_Asset resolution_ is the process by which an [asset path](https://openusd.org/release/glossary.html#usdglossary-asset)
is translated into the location of a consumable resource. USD provides a plugin
point for asset resolution, the ArResolver interface, which clients can
implement to resolve assets discovered in a USD scene, using whatever logic and
external inputs they require. This plugin also lets clients provide USD with
an asset’s data without fetching the data to disk.

If no asset resolver plugin is available, USD falls back to a default resolver
implementation that uses search paths to locate assets referenced via relative
paths. See the examples for [References](https://openusd.org/release/glossary.html#usdglossary-references).

## [Attribute](https://openusd.org/release/glossary.html\#id371) [](https://openusd.org/release/glossary.html\#attribute "Permalink to this heading")

_Attributes_ are the most common type of [property](https://openusd.org/release/glossary.html#usdglossary-property)
authored in most USD scenes. An attribute can take on exactly one of the legal
[attribute typeNames](https://openusd.org/release/api/_usd__page__datatypes.html)
USD provides, and can take on both a [default value](https://openusd.org/release/glossary.html#usdglossary-defaultvalue) **and** a value each at any number of [timeSamples](https://openusd.org/release/glossary.html#usdglossary-timesample).
[Resolving](https://openusd.org/release/glossary.html#usdglossary-valueresolution) an attribute at any given
[timeCode](https://openusd.org/release/glossary.html#usdglossary-timecode) will yield either a single value or no
value. Attributes resolve according to “strongest wins” rules, so all values
for any given attribute will be fetched from the strongest [PrimSpec](https://openusd.org/release/glossary.html#usdglossary-primspec) that provides **either** a default value or
timeSamples. Note that this simple rule is somewhat more complicated in the
presence of authored [clips](https://openusd.org/release/glossary.html#usdglossary-valueclips). One interacts with
attributes through the [UsdAttribute](https://openusd.org/release/api/class_usd_attribute.html) API.

A simple example of an attribute that has both an authored default and two
timeSamples in the same primSpec:

An attribute with both Default and TimeSamples [](https://openusd.org/release/glossary.html#id314 "Permalink to this code")

```
def Sphere "BigBall"
{
    double radius = 100
    double radius.timeSamples = {
        1: 100,
        24: 500,
    }
}

```

## [Attribute Block](https://openusd.org/release/glossary.html\#id372) [](https://openusd.org/release/glossary.html\#attribute-block "Permalink to this heading")

Similarly to how [prims can be deactivated](https://openusd.org/release/glossary.html#usdglossary-active-inactive)
through composing overriding opinions, the value that an attribute produces can
be **blocked** by an overriding opinion of None, which can be authored using
[UsdAttribute::Block](https://openusd.org/release/api/class_usd_attribute.html#a88e75995833fc8cb897d9ed123c3f3e0). A block itself can, of course be overridden by
an even stronger opinion. The following example extends the previous attribute
example, adding a DefaultBall prim that blocks the value of
radius it references from BigBall, causing radius’s
value to resolve back to [its fallback](https://openusd.org/release/glossary.html#usdglossary-fallback) at
UsdTimeCodet (any blocked attribute that has no fallback will
report that it has no value when we invoke [UsdAttribute::Get](https://openusd.org/release/api/class_usd_attribute.html#a9d41bc223be86408ba7d7f74df7c35a9)):

DefaultBall **Blocks** the radius values referenced from BigBall [](https://openusd.org/release/glossary.html#id315 "Permalink to this code")

```
def Sphere "BigBall"
{
    double radius = 100
    double radius.timeSamples = {
        1: 100,
        24: 500,
    }
}

def "DefaultBall" (
    references = </BigBall>
)
{
    double radius = None
}

```

In addition to completely blocking an attribute’s value, sub time-ranges can be
separately blocked, by blocking individual time samples. Consider the following
examples:

**Example 1:**

```
def Sphere "BigBall"
{
    double radius.timeSamples = {
        101: 12,
        102: None,
    }
}

```

For the attribute radius on BigBall:

> - Usd.Attribute.Get(t) will return 12 for Usd.TimeCodet in (-∞, 102).
>
> - Usd.Attribute.Get(t) will return None for
> Usd.TimeCodet in \[102, ∞).\
\
**Example 2:**\
\
```\
def Sphere "BigBall"\
{\
    double radius.timeSamples = {\
        101: None,\
        102: 12,\
    }\
}\
\
```\
\
For the attribute radius on BigBall:\
\
> - Usd.Attribute.Get(t) will return None for\
> Usd.TimeCodet in (-∞, 102).\
>\
> - Usd.Attribute.Get(t) will return 12 for Usd.TimeCodet in \[102, ∞).\
\
Note that the per-timeSample-blocking ability does **not** allow us to sparsely\
**override** timeSamples, i.e. in the following example:\
\
```\
def Sphere "BigBall"\
{\
    double radius.timeSamples = {\
        101: 1,\
        102: 2,\
    }\
}\
\
def "DefaultBall" (\
    references = </BigBall>\
)\
{\
    double radius.timeSamples = {\
        101: None,\
    }\
}\
\
```\
\
For the attribute radius on DefaultBall:\
\
> - Usd.Attribute.Get(t) will return None for\
> Usd.TimeCodet in (-∞, ∞).\
\
## [Attribute Connection](https://openusd.org/release/glossary.html\#id373) [](https://openusd.org/release/glossary.html\#attribute-connection "Permalink to this heading")\
\
See [Connection](https://openusd.org/release/glossary.html#usdglossary-connection).\
\
## [Attribute Variability](https://openusd.org/release/glossary.html\#id374) [](https://openusd.org/release/glossary.html\#attribute-variability "Permalink to this heading")\
\
See [Variability](https://openusd.org/release/glossary.html#usdglossary-variability).\
\
## [Change Processing](https://openusd.org/release/glossary.html\#id375) [](https://openusd.org/release/glossary.html\#change-processing "Permalink to this heading")\
\
_Change processing_ is the action a [UsdStage](https://openusd.org/release/api/class_usd_stage.html) takes in response to edits\
of any of the layers that contribute to its composition. During change\
processing, prims on the stage may be [re-indexed](https://openusd.org/release/glossary.html#usdglossary-index) or\
disappear entirely, or new prims may come into being. The key things to\
understand about change processing are:\
\
1. It is always active. Whenever you use the Usd or Sdf APIs to\
mutate any layers that contribute to a UsdStage’s composition, that\
stage will update itself immediately, in the same thread in which authoring\
was performed (though the change-processing itself may spawn worker threads),\
so that it always presents an accurate result to clients.\
\
2. When a stage has completed a round of change processing, it will\
[send notification](https://openusd.org/release/api/class_usd_notice.html) to clients who have registered\
interest, so that they may keep themselves updated in light of authored\
changes to the stage.\
\
\
\
Note\
\
\
\
> Clients listening to [UsdStage](https://openusd.org/release/api/class_usd_stage.html) notifications should not update\
> themselves immediately upon receiving a notice, but instead note what has\
> become out-of-date (dirty), and defer updating until necessary; this helps\
> minimize the amount of work an application needs to undertake when many\
> edits are being performed rapidly.\
\
3. For all of the layers that contribute to a given [UsdStage](https://openusd.org/release/api/class_usd_stage.html), **only**\
**one thread at a time may be editing any of those layers, and no other thread**\
**can be reading from the stage while the editing and change processing are**\
**taking place.** This is because we do not want to weigh down read access to\
[UsdStage](https://openusd.org/release/api/class_usd_stage.html) data with any kind of locking.\
\
\
## [Class](https://openusd.org/release/glossary.html\#id376) [](https://openusd.org/release/glossary.html\#class "Permalink to this heading")\
\
_Class_ is one of the three possible [specifiers](https://openusd.org/release/glossary.html#usdglossary-specifier) a\
[prim](https://openusd.org/release/glossary.html#usdglossary-prim) (and also a [primSpec](https://openusd.org/release/glossary.html#usdglossary-primspec)) can\
possess. A class prim and all of the prims nested inside it in [namespace](https://openusd.org/release/glossary.html#usdglossary-namespace) will report that they are **abstract**, via\
[UsdPrim::IsAbstract](https://openusd.org/release/api/class_usd_prim.html#afcaf9f166343aca6a5b6cf5764947720), which causes the prims to be skipped by stage and\
child traversals, unless a client specifically asks to include abstract\
prims. The most common use of classes is to create prims from which other prims\
can [inherit](https://openusd.org/release/glossary.html#usdglossary-inherits). Classes can define/override metadata as\
well as properties and nested prims. The following example contains a class\
\_class\_Ball that provides a value for the radius attribute for any prim\
that would [inherit](https://openusd.org/release/glossary.html#usdglossary-inherits) or [reference](https://openusd.org/release/glossary.html#usdglossary-references) it. The \_class\_ prefix is a Pixar convention\
for naming classes, and is not a requirement.\
\
A “class” prim contains opinions meant to be inherited [](https://openusd.org/release/glossary.html#id316 "Permalink to this code")\
\
```\
class "_class_Ball" {\
    double radius = 50\
}\
\
```\
\
## [Clips](https://openusd.org/release/glossary.html\#id377) [](https://openusd.org/release/glossary.html\#usdglossary-clips "Permalink to this heading")\
\
See [Value Clips](https://openusd.org/release/glossary.html#usdglossary-valueclips)\
\
## [Collection](https://openusd.org/release/glossary.html\#id378) [](https://openusd.org/release/glossary.html\#collection "Permalink to this heading")\
\
Collections build on the ability of [relationships](https://openusd.org/release/glossary.html#usdglossary-relationship)\
to identify objects in a USD scene. Whereas a relationship is (resolves to, by\
fetching its _Targets_) simply an ordered list of paths identifying other\
objects in the scene, a Collection uses a pair of relationships and extra rules\
to compactly encode potentially large sets of objects by identifying a set of\
paths to hierarchically **include** in the Collection, and a separate set of\
paths to hierarchically **exclude**. For example, the following Collection\
identifies all of the prims comprising the two buildings, _except_ for all the\
prims organized under the Floor13 prim in each.\
\
Collections refine a hierarchical set by including and excluding\
finer and finer branches [](https://openusd.org/release/glossary.html#id317 "Permalink to this code")\
\
```\
over "World"\
{\
    over "Buildings" (\
        prepend apiSchemas = "CollectionAPI:luckyBuildings"\
    )\
    {\
        uniform token collection:luckyBuildings:expansionRule = "expandPrims"\
        rel collection:luckyBuildings:includes = [\
            </World/Buildings/Skyscraper>,\
            </World/Buildings/Pyramid>,\
        ]\
        rel collection:luckyBuildings:excludes = [\
            </World/Buildings/Skyscraper/Floor13>,\
            </World/Buildings/Pyramid/Floor13>,\
        ]\
    }\
}\
\
```\
\
We can see from the example that Collections in USD are expressed as “multiple\
apply” [API Schemas](https://openusd.org/release/glossary.html#usdglossary-apischema), so that we can add as many\
different collections as we want, to any prim already in the scene. The\
expansionRule attribute specifies how the targets of the\
includes relationship should be expanded; we can include all prims, all\
prims and properties, or just the targeted objects themselves. In addition to\
prims and properties, a Collection can include _another Collection,_ which\
allows us to hide internal details of assets when we publish them, as the\
aggregate scenes that consume the assets can do things like binding materials\
and illuminating the scene by targeting the asset’s collections (which break\
down the asset into meaningful groups) with other collections.\
\
We create and query Collections using [UsdCollectionAPI](https://openusd.org/release/api/class_usd_collection_a_p_i.html).\
\
For more details on Collections, including pattern-based collections that use\
a path expression to determine collection membership,\
see [Collections and Patterns](https://openusd.org/release/user_guides/collections_and_patterns.html#collections-and-patterns).\`\
\
## [Component](https://openusd.org/release/glossary.html\#id379) [](https://openusd.org/release/glossary.html\#component "Permalink to this heading")\
\
A _component_ is a “leaf” [kind](https://openusd.org/release/glossary.html#usdglossary-kind) of [Model](https://openusd.org/release/glossary.html#usdglossary-model). Components can contain [subcomponents](https://openusd.org/release/glossary.html#usdglossary-subcomponent), but no other models. Components _can_ reference\
in other assets that are published as models, but they should override the\
kind of the referenced prim to “subcomponent”.\
\
A “component” asset, declared in scene description, overriding the\
kind of a “nested” asset reference [](https://openusd.org/release/glossary.html#id318 "Permalink to this code")\
\
```\
def Xform "TreeSpruce" (\
    kind = "component"\
)\
{\
    # Geometry and shading prims that define a Spruce tree...\
\
    def "Cone_1" (\
        kind = "subcomponent"\
        references = @Cones/PineConeA.usd@\
    )\
    {\
    }\
}\
\
```\
\
**See also:** [model hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy)\
\
## [Composition](https://openusd.org/release/glossary.html\#id380) [](https://openusd.org/release/glossary.html\#composition "Permalink to this heading")\
\
_Composition_ is the process that assembles multiple [layers](https://openusd.org/release/glossary.html#usdglossary-layer) together by the [composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) that relate them to each other, resulting in a\
[UsdStage](https://openusd.org/release/glossary.html#usdglossary-stage) scenegraph of [UsdPrims](https://openusd.org/release/glossary.html#usdglossary-prim). Each UsdPrim contains an [index](https://openusd.org/release/glossary.html#usdglossary-index) that allows clients to subsequently extract “ [resolved\\
values](https://openusd.org/release/glossary.html#usdglossary-valueresolution)” for properties and metadata from the\
relevant layers. Composition occurs when first opening a UsdStage,\
when [loading or unloading prims](https://openusd.org/release/glossary.html#usdglossary-load-unload) on the stage, and\
when [layers that contribute to the stage are edited](https://openusd.org/release/glossary.html#usdglossary-changeprocessing). One of the results of composition is [path\\
translation](https://openusd.org/release/glossary.html#usdglossary-pathtranslation) so that all the data in all layers\
contributing to the scene are accessed by their “scene level” namespace\
locations (paths).\
\
We also sometimes refer to “a composition” or “a composed prim” or “a composed\
scene”, in which contexts we are referring to the _result_ of performing\
composition.\
\
## [Composition Arcs](https://openusd.org/release/glossary.html\#id381) [](https://openusd.org/release/glossary.html\#usdglossary-compositionarcs "Permalink to this heading")\
\
_Composition arcs_ are the “operators” that allow USD to create rich\
compositions of many layers containing mixes of “base” scene description and\
overrides. The arcs are:\
\
> - [subLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers)\
>\
> - [inherits](https://openusd.org/release/glossary.html#usdglossary-inherits)\
>\
> - [variantSets](https://openusd.org/release/glossary.html#usdglossary-variantset)\
>\
> - [references](https://openusd.org/release/glossary.html#usdglossary-references)\
>\
> - [payloads](https://openusd.org/release/glossary.html#usdglossary-payload)\
>\
> - [specializes](https://openusd.org/release/glossary.html#usdglossary-specializes)\
\
We refer to these operators as **arcs** because each one targets either a [layer](https://openusd.org/release/glossary.html#usdglossary-layer), a [prim](https://openusd.org/release/glossary.html#usdglossary-prim), or a combination of layer\
and prim, and when diagramming a [prim’s index](https://openusd.org/release/glossary.html#usdglossary-index) to\
understand how a scene or value is composed, the composition operators represent\
the directional arcs that combine layers and [primSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec) into an ordered graph that we traverse when performing\
[value resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution). Except for subLayers, all\
composition arcs target a specific prim in a LayerStack, and allow the renaming\
of that target prim as the result “flows” across the arc. USD performs all the\
necessary [path translation](https://openusd.org/release/glossary.html#usdglossary-pathtranslation) to ensure that\
stage-level queries always work in the stage-level namespace, regardless of how\
many different and/or nested composition arcs and name-changes contributed to\
the result.\
\
Except for subLayers, all composition arcs are [list editable](https://openusd.org/release/glossary.html#usdglossary-listediting), which means that each layer in a [layerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack) can sparsely prepend, append, remove, or reset\
targets, which allows us to non-destructively edit the composition structure of\
a scene as it evolves or passes through multiple stages of a pipeline. Following\
is an example of list-edited references. The **resolved** value of references on\
/MyPrim that will be consumed during [composition](https://openusd.org/release/glossary.html#usdglossary-composition) of superLayer.usd is a two-element\
list: \[ @file1.usd@, @file3.usd@ \]\
\
Contents of file base.usd [](https://openusd.org/release/glossary.html#id319 "Permalink to this code")\
\
```\
#usda 1.0\
\
def "MyPrim" (\
    references = [\
        @file1.usd@,\
        @file2.usd@\
    ]\
)\
{\
}\
\
```\
\
Contents of file superLayer.usd [](https://openusd.org/release/glossary.html#id320 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    subLayers = [\
        @./base.usd@\
    ]\
)\
\
# Removes reference to file2.usd, while adding a reference to file3.usd at the end of the list\
over "MyPrim" (\
    delete references = [ @file2.usd@ ]\
    append references = [ @file3.usd@ ]\
)\
{\
}\
\
```\
\
## [Connection](https://openusd.org/release/glossary.html\#id382) [](https://openusd.org/release/glossary.html\#usdglossary-connection "Permalink to this heading")\
\
Connections are quite similar to [relationships](https://openusd.org/release/glossary.html#usdglossary-relationship)\
in that they are list-edited “scene pointers” that robustly identify other\
scene objects. The key difference is that while relationships are typeless,\
independent [properties](https://openusd.org/release/glossary.html#usdglossary-property), connections are instead a\
sub-aspect of USD [attributes](https://openusd.org/release/glossary.html#usdglossary-attribute). Relationships serve\
to establish dependencies between prims as a whole, or the dependency of a prim\
as a whole upon a targeted property. But they lack the expressiveness to\
encode, for example, complex, typed, dataflow networks such as shading\
networks. In networks in which each node has multiple, value-typed inputs\
and/or outputs, we can represent the nodes as prims, and the inputs and outputs\
as attributes. Each attribute is strongly typed, and _connections_ allow each\
input to target an output attribute from which, in some downstream consuming\
application, it may receive dataflow.\
\
Connections empower USD to robustly encode dataflow networks of prims, but **USD**\
**itself provides no dataflow behavior across connections.** Instead, USD\
stipulates that [schemas](https://openusd.org/release/glossary.html#usdglossary-schema) can impose semantics on\
connections, to be interpreted/implemented by the schema or higher-level\
systems. We do so for two reasons:\
\
> 1. Consulting connections during [value resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution)(UsdAttribute::Get()) would\
> necessarily slow down _all_ attribute value resolution, whether the\
> attribute possesses connections or not.\
>\
> 2. Interchange is an important aspect of USD, and we are not currently\
> willing to tackle the conformance problem of different dataflow semantics\
> between different DCCs.\
\
A basic example of connections, from the [Simple Shading in USD](https://openusd.org/release/tut_simple_shading.html)\
tutorial, which demonstrates how USD’s shading model uses input-to-output\
connections to indicated render-time dataflow between shaders, and\
output-to-output connections on Materials to bind shader outputs to important\
computed results consumed by the renderer:\
\
```\
def Material "boardMat"\
{\
    token inputs:frame:stPrimvarName = "st"\
    token outputs:surface.connect = </TexModel/boardMat/PBRShader.outputs:surface>\
\
    def Shader "PBRShader"\
    {\
        uniform token info:id = "UsdPreviewSurface"\
        color3f inputs:diffuseColor.connect = </TexModel/boardMat/diffuseTexture.outputs:rgb>\
        float inputs:metallic = 0\
        float inputs:roughness = 0.4\
        token outputs:surface\
    }\
\
    def Shader "stReader"\
    {\
        uniform token info:id = "UsdPrimvarReader_float2"\
        token inputs:varname.connect = </TexModel/boardMat.inputs:frame:stPrimvarName>\
        float2 outputs:result\
    }\
\
    def Shader "diffuseTexture"\
    {\
        uniform token info:id = "UsdUVTexture"\
        asset inputs:file = @USDLogoLrg.png@\
        float2 inputs:st.connect = </TexModel/boardMat/stReader.outputs:result>\
        float3 outputs:rgb\
    }\
}\
\
```\
\
## [Crate File Format](https://openusd.org/release/glossary.html\#id383) [](https://openusd.org/release/glossary.html\#crate-file-format "Permalink to this heading")\
\
The _crate_ file format is USD’s own binary file format, whose file extension is\
.usdc, and which is losslessly, bidirectionally convertible to the\
.usda text format. The .usd file format is special, as\
files with that extension can be **either** crate or text files; this\
facilitates debugging as crate files can be converted to text in-place for\
rapid iterative hand-editing without needing to change any referencing\
layers. The primary differences between text and crate files (other than the\
obvious human readability aspect) are:\
\
> 1. Text files must be read in their entirety, parsed, and stored in-memory\
> as soon as they are opened, whereas crate files read in only a small\
> index of the file’s contents when they are opened, deferring access to\
> big data until a client requests it specifically.\
>\
> 2. Except for very small files (under a few hundred kilobytes), crate files\
> will be much more compact than text files.\
\
Crate was designed for low-latency and high-performance lazy queries, and we\
believe it to be the best file format choice for storing scene description\
consumed by USD. Some of its features include:\
\
> - Aggressive, multi-level data deduplication yields small file sizes\
>\
> - Lockless data extraction for high-bandwidth multi-threaded reading\
>\
> - Access to files either by mmap() or pread(), trading VM\
> pressure for file descriptor consumption and system call overhead. By\
> default crate uses mmap(), but the choice is configurable at\
> runtime.\
>\
> - Low latency in “cold file-system cache” network access, as all data needed\
> to open a crate file for USD’s use is compacted into a contiguous footer\
> section.\
>\
> - Editing crate files does not copy all data to a new file. Instead, it\
> appends. Disused values consume disk space, so repeated editing may\
> produce files larger than ideal. Use [usdcat](https://openusd.org/release/toolset.html#usdcat) to rewrite\
> files in their most compact form.\
\
You can [convert between file formats using usdcat](https://openusd.org/release/tut_converting_between_layer_formats.html).\
\
## [Def](https://openusd.org/release/glossary.html\#id384) [](https://openusd.org/release/glossary.html\#def "Permalink to this heading")\
\
_Def_ is one of the three possible [specifiers](https://openusd.org/release/glossary.html#usdglossary-specifier) a\
[prim](https://openusd.org/release/glossary.html#usdglossary-prim) (and also a [primSpec](https://openusd.org/release/glossary.html#usdglossary-primspec))\
can possess. A _def_ **defines** a prim, which, for most consumers of USD,\
equates to the prim being present on the stage and available for\
processing. Prims whose specifier resolves to [class](https://openusd.org/release/glossary.html#usdglossary-class)\
or [over](https://openusd.org/release/glossary.html#usdglossary-over) are actually present and composed on a stage,\
but will not be visited by [UsdPrim::GetChildren](https://openusd.org/release/api/class_usd_prim.html#a2619563fc9180d8ead597944fd7f6ec7) or\
[UsdStage::Traverse](https://openusd.org/release/api/class_usd_stage.html#adba675b55f41cc1b305bed414fc4f178). A _def_ may, but need not declare a schema type\
for the prim. For further information see the FAQ: [What’s the difference between an “over” and a “typeless def” ?](https://openusd.org/release/usdfaq.html#what-s-the-difference-between-an-over-and-a-typeless-def)\
\
The following example defines a prim /Ball as belonging to the **Sphere**\
schema, and sets its radius to 50.\
\
A “def” defines a prim [](https://openusd.org/release/glossary.html#id321 "Permalink to this code")\
\
```\
def Sphere "Ball" {\
    double radius = 50\
}\
\
```\
\
## [Default Value](https://openusd.org/release/glossary.html\#id385) [](https://openusd.org/release/glossary.html\#usdglossary-defaultvalue "Permalink to this heading")\
\
Many assets consist entirely of a static (with respect to time) definition,\
which really exists “outside time”. When encoding such assets in a format that\
only allows [timeSamples](https://openusd.org/release/glossary.html#usdglossary-timesample), one must choose a\
“sentinel” time coordinate at which to record static data, and hope that no other\
application uses that sentinel time for any other purpose. This can be fragile,\
and also lead to the “static” definition becoming overshadowed and not easily\
accessible when overridden in a stronger layer.\
\
USD addresses this problem by providing a completely separate field for each\
attribute, called its _default_. This field can be authored and resolved in\
isolation of any authored timeSamples anywhere in an attribute’s [index](https://openusd.org/release/glossary.html#usdglossary-index), by using the universal, reserved sentinel\
[UsdTimeCode::Default](https://openusd.org/release/api/class_usd_time_code.html#a8a2192045dc22e90fe08ef2d8d68f3b8) as the (implicit or explicit) time coordinate to\
[UsdAttribute::Get](https://openusd.org/release/api/class_usd_attribute.html#a9d41bc223be86408ba7d7f74df7c35a9) and [UsdAttribute::Set](https://openusd.org/release/api/class_usd_attribute.html#a7fd0957eecddb7cfcd222cccd51e23e6). When\
[resolving an attribute’s value](https://openusd.org/release/glossary.html#usdglossary-valueresolution) at a\
non-Default time, defaults still participate, but within a given [primSpec](https://openusd.org/release/glossary.html#usdglossary-primspec), an authored default is always weaker than authored\
timeSamples. However, an authored default in a stronger layer/primSpec **is**\
**stronger** than timeSamples authored in a weaker layer. In text USD layers, the\
default value is the single value that can be assigned directly to an attribute\
in the attribute declaration line:\
\
Overriding the default value of a Ball’s radius [](https://openusd.org/release/glossary.html#id322 "Permalink to this code")\
\
```\
over "Ball" {\
    double radius = 50\
}\
\
```\
\
## [Direct Opinion](https://openusd.org/release/glossary.html\#id386) [](https://openusd.org/release/glossary.html\#direct-opinion "Permalink to this heading")\
\
A _direct opinion_ for some property or metadatum of a prim at path\
/foo/bar/baz in a [layerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack) is one that\
is authored “directly” on the [primSpec](https://openusd.org/release/glossary.html#usdglossary-primspec) at\
/foo/bar/baz **in contrast** to an indirect opinion on a different\
primSpec that affects /foo/bar/baz due to the effect of one or more\
composition arcs. Two examples of where we find it useful to talk about direct\
vs. indirect opinions are:\
\
> 1. [References](https://openusd.org/release/glossary.html#usdglossary-references).\
>\
> A _direct reference_ is one authored on a prim itself, as opposed to an\
> _ancestral reference_ , authored on some ancestor of the prim. Ancestral\
> references are weaker than direct references, so if the targets of both\
> the direct and ancestral references contain opinions about the same\
> property on the prim, the opinions of the direct reference will win. This\
> “weaker ancestor” behavior is also true for direct vs ancestral [Payloads](https://openusd.org/release/glossary.html#usdglossary-payload), [VariantSets](https://openusd.org/release/glossary.html#usdglossary-variantset),\
> [Inherits](https://openusd.org/release/glossary.html#usdglossary-inherits), and [Specializes](https://openusd.org/release/glossary.html#usdglossary-specializes) arcs.\
>\
> 2. [Specializes arcs](https://openusd.org/release/glossary.html#usdglossary-specializes).\
>\
> When prim /root/derived specializes prim /root/base\
> in a layer, _direct opinions_ authored on the “derived” prim in any\
> referencing context (that is, a layerStack that references the\
> /root prim in the original layer, **or** any layerStack that\
> references the new layerStack, ad infinitum) will always be stronger than\
> any opinions expressed directly on the “base” prim in any of the\
> referencing contexts. See [specializes](https://openusd.org/release/glossary.html#usdglossary-specializes) for\
> examples.\
\
## [EditTarget](https://openusd.org/release/glossary.html\#id387) [](https://openusd.org/release/glossary.html\#edittarget "Permalink to this heading")\
\
When authoring composed scene description, it is often desirable to edit the\
targets of various [composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) _in_\
_context_ of the scene you are constructing, rather than needing to edit\
individual layers in isolation. _Edit Targets_, embodied by the\
[UsdEditTarget](https://openusd.org/release/api/class_usd_edit_target.html) class, allow you to work with the composed stage and the\
[UsdPrims](https://openusd.org/release/api/class_usd_prim.html) it contains, while specifying which contributing\
site in the stage’s network of composition arcs should receive the opinions you\
are about to author using the composed prim. You can think of Edit Targets as an\
extension of the idea of “selecting a layer” in\
Photoshop. [UsdEditTarget](https://openusd.org/release/api/class_usd_edit_target.html) provides specific methods for selecting any\
[sublayer](https://openusd.org/release/glossary.html#usdglossary-sublayers) in a stage’s [root layerStack](https://openusd.org/release/glossary.html#usdglossary-rootlayerstack), or the currently selected [variant](https://openusd.org/release/glossary.html#usdglossary-variant) of any defined top-level or nested [variantSet](https://openusd.org/release/glossary.html#usdglossary-variantset). In addition, You can create an EditTarget from any\
“Node” in a prim’s [PrimIndex](https://openusd.org/release/glossary.html#usdglossary-index), which allows you to target\
[inherited classes](https://openusd.org/release/glossary.html#usdglossary-inherits), [reference targets](https://openusd.org/release/glossary.html#usdglossary-references), etc.\
\
## [Fallback](https://openusd.org/release/glossary.html\#id388) [](https://openusd.org/release/glossary.html\#fallback "Permalink to this heading")\
\
Many [IsA Schemas](https://openusd.org/release/glossary.html#usdglossary-isaschema) and applied [API Schemas](https://openusd.org/release/glossary.html#usdglossary-apischema) define attributes that have an identifiable value\
that makes sense in most situations. The [USD schema generation process](https://openusd.org/release/tut_generating_new_schema.html) allows a schema creator to specify such\
values as a _fallback_ that will be implicitly present on an attribute even when\
no values have been authored for it. The presence of fallback values allows us\
to keep our scene description sparse and compact, and allows for\
self-documenting behavior. Fallbacks are deployed extensively in the [UsdGeom\\
schemas](https://openusd.org/release/api/usd_geom_page_front.html), for example\
[UsdGeomImageable’s visibility attribute](https://openusd.org/release/api/class_usd_geom_imageable.html#a943eddc325190bd67789407400f122b7) has a fallback value of\
inherited, and [UsdGeomGprim’s orientation attribute](https://openusd.org/release/api/class_usd_geom_gprim.html#a6d7f451cf0aa27125cc118b030ebb735) has a fallback of rightHanded.\
\
## [Flatten](https://openusd.org/release/glossary.html\#id389) [](https://openusd.org/release/glossary.html\#flatten "Permalink to this heading")\
\
Even though USD derives great efficiencies from accessing data directly from the\
[layers](https://openusd.org/release/glossary.html#usdglossary-layer) that a [stage](https://openusd.org/release/glossary.html#usdglossary-stage) composes\
together as directed by the various [composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) that weave the files together, it can sometimes\
be useful to “bake down the composition” into a single layer that no longer\
contains any composition arcs. A flattened stage is highly portable since its\
single layer is self-contained, and in some cases, it _may_ be more efficient to\
compose and resolve, although this is definitely not a given. For example,\
flattening a stage to an text USD layer will generally produce extremely large\
files since assets that were referenced multiple times on a stage will be\
uniquely baked out into their own namespaces, with all data duplicated; the\
[crate file format](https://openusd.org/release/glossary.html#usdglossary-cratefileformat) will perform better in this\
metric, at least, since it performs data deduplication. Regardless of file\
format, the action of flattening a stage will generally be memory and\
compute-intensive, and will not, at this time, benefit from multithreading.\
\
To flatten a stage, use [UsdStage::Flatten](https://openusd.org/release/api/class_usd_stage.html#ae3271778fa2ceeb82fbd51296610820a) or [usdcat](https://openusd.org/release/toolset.html#usdcat)\
with the `--flatten` option.\
\
To flatten individual [layer stacks](https://openusd.org/release/glossary.html#usdglossary-layerstack), use\
[UsdUtilsFlattenLayerStack](https://openusd.org/release/api/flatten_layer_stack_8h.html#a82cccc4bb061ba2244168e4a2515a6c6) or [usdcat](https://openusd.org/release/toolset.html#usdcat) with the\
`--flattenLayerStack` option.\
\
## [Gprim](https://openusd.org/release/glossary.html\#id390) [](https://openusd.org/release/glossary.html\#usdglossary-gprim "Permalink to this heading")\
\
_Gprim_ comes from [Pixar’s RenderMan](https://renderman.pixar.com/)\
terminology, and is a contraction for “Geometric primitive”, which is to say,\
any primitive whose imaging/rendering will directly cause something to be\
drawn. Gprim is a first-class concept in USD, embodied in the class\
[UsdGeomGprim](https://openusd.org/release/api/class_usd_geom_gprim.html). All Gprims possess the following qualities:\
\
> - Gprims are [boundable](https://openusd.org/release/api/class_usd_geom_boundable.html#details),\
> and should always submit to computing an extent (even if it be an empty\
> extent), and valid UsdGeom scene description should **always**\
> **provide an authored extent** on a gprim that captures its changing shape\
> (if its shape is animated).\
>\
> - Gprims are directly [transformable](https://openusd.org/release/api/class_usd_geom_xformable.html#details) ,\
> which is the primary distinguishing factor between [UsdGeomGprim](https://openusd.org/release/api/class_usd_geom_gprim.html)\
> and Autodesk Maya’s similar “shape” node type. Transformable gprims\
> necessitate fewer prims overall in most 3D compositions, which aids\
> scalability, since prim-count is one of the primary scaling factors for\
> USD.\
\
Effectively structuring gprims in namespace\
\
Please observe the following two rules when laying out gprims in a namespace\
hierarchy:\
\
1. **Do not nest gprims in** [namespace](https://openusd.org/release/glossary.html#usdglossary-namespace).\
\
We consider it invalid to nest gprims under other gprims, and\
[usdchecker](https://openusd.org/release/toolset.html#usdchecker) will issue warnings on scenes\
that contain this construct. This is because key features of USD apply\
hierarchically and are “pruning”, such as [activation](https://openusd.org/release/glossary.html#usdglossary-active-inactive) and [visibility](https://openusd.org/release/glossary.html#usdglossary-visibility). When an ancestor gprim is deactivated or\
made invisible, there is no possible way to make any descendant gprim\
active or visible.\
\
2. **Do not directly** [instance](https://openusd.org/release/glossary.html#usdglossary-instancing) **gprims.**\
\
Because the root-prims of instance prototypes possess no properties, it\
is pointless to instance a gprim directly. Renderers _must not_\
infer instanceability from an instance of a gprim prototype, because each\
instance is allowed to override any property defined originally on the\
referenced prototype root prim. One can use USD’s instancing feature to\
create gprim-level instancing, but to do so requires adding an Xform\
parent to the gprim in the prototype, and making the instances reference\
the parent, rather than the gprim, directly.\
\
\
## [Group](https://openusd.org/release/glossary.html\#id391) [](https://openusd.org/release/glossary.html\#usdglossary-group "Permalink to this heading")\
\
In USD a _group_ is a [kind](https://openusd.org/release/glossary.html#usdglossary-kind) of [Model](https://openusd.org/release/glossary.html#usdglossary-model). Groups are models that aggregate other models into\
meaningful collections. Whereas the specialized group-model kind [assembly](https://openusd.org/release/glossary.html#usdglossary-assembly) generally identifies group models that are published\
[assets](https://openusd.org/release/glossary.html#usdglossary-asset), groups tend to be simple “inlined” model prims\
defined inside and as part of assemblies. They are the “glue” that holds a\
[model hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy) together.\
\
An “assembly” asset, that contains “group” models for organizing its sub-parts [](https://openusd.org/release/glossary.html#id323 "Permalink to this code")\
\
```\
def Xform "Forest_set" (\
    kind = "assembly"\
)\
{\
    def Xform "Outskirts" (\
        kind = "group"\
    )\
    {\
        # More deeply nested groups, bottoming out at references to other assemblies and components\
    }\
\
    def Xform "Glade" (\
        kind = "group"\
    )\
    {\
        # More deeply nested groups, bottoming out at references to other assemblies and components\
    }\
}\
\
```\
\
**See also:** [model hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy)\
\
## [Hydra](https://openusd.org/release/glossary.html\#id392) [](https://openusd.org/release/glossary.html\#hydra "Permalink to this heading")\
\
_Hydra_ is a modern rendering architecture optimized for handling very large\
scenes and “change processing” (i.e. responding to authored or time-varying\
changes to the scene inputs). It has three major components: the _scene_\
_delegates_ (which provide scene data), the _render index_ (responsible for\
change tracking and other scene management), and the _render delegates_ (which\
consume the scene data to produce an image). This flexible architecture allows\
for easy integration within pipelines that have their own scene data, as well\
as their own renderers. Pixar uses Hydra for asset preview in many of its\
tools, including **usdview** and the [Presto Animation System](http://www.cartoonbrew.com/tech/watch-a-rare-demo-of-pixars-animation-system-presto-98099.html).\
\
If you’re interested in developing with Hydra components, see the\
[Hydra Getting Started Guide](https://openusd.org/release/api/_page__hydra__getting__started__guide.html)\
and other Hydra developer documentation in our [Developer Guides](https://openusd.org/release/api/_developer__guides.html).\
\
## [Index](https://openusd.org/release/glossary.html\#id393) [](https://openusd.org/release/glossary.html\#usdglossary-index "Permalink to this heading")\
\
An _index_, also referred to as a [PrimIndex](https://openusd.org/release/api/class_pcp_prim_index.html),\
is the result of [composition](https://openusd.org/release/glossary.html#usdglossary-composition), and the primary\
piece of information we compute and cache for a composed [prim](https://openusd.org/release/glossary.html#usdglossary-prim) on a [stage](https://openusd.org/release/glossary.html#usdglossary-stage). A prim’s index\
contains an ordered (from strongest to weakest) list of “Nodes”, which are all\
of the locations in [layers](https://openusd.org/release/glossary.html#usdglossary-layer) (also known as [primSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec)) that contribute opinions to the data contained in the\
composed prim, as well as an indication of how each location was woven into the\
composite, i.e. what [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) was\
traversed to discover it.\
\
All of the queries on USD classes except for stage-level metadata rely on prim\
indices to perform [value resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution). USD\
also uses prim indices to compute [primStacks](https://openusd.org/release/glossary.html#usdglossary-primstack) for\
debugging, and to construct [Edit Targets](https://openusd.org/release/glossary.html#usdglossary-edittarget).\
\
## [Inherits](https://openusd.org/release/glossary.html\#id394) [](https://openusd.org/release/glossary.html\#usdglossary-inherits "Permalink to this heading")\
\
_Inherits_ is a [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) that addresses\
the problem of adding a single, non-destructive edit (override) that can affect\
a whole _class_ of distinct objects on a [stage](https://openusd.org/release/glossary.html#usdglossary-stage). Inherits\
acts as a non-destructive “broadcast” operator that applies opinions authored\
on one prim to every other prim that inherits the “source” prim; not only do\
property opinions broadcast over inherits arcs - **all** scene description,\
hierarchically from the source, inherits. Consider the following example:\
\
Trees.usd, demonstrating inherits [](https://openusd.org/release/glossary.html#id324 "Permalink to this code")\
\
```\
#usda 1.0\
\
class Xform "_class_Tree"\
{\
    def Mesh "Trunk"\
    {\
        color3f[] primvars:displayColor = [(.8, .8, .2)]\
    }\
\
    def Mesh "Leaves"\
    {\
        color3f[] primvars:displayColor = [(0, 1, 0)]\
    }\
}\
\
def "TreeA" (\
    inherits = </_class_Tree>\
)\
{\
}\
\
def "TreeB" (\
    inherits = </_class_Tree>\
)\
{\
    over "Leaves"\
    {\
        color3f[] primvars:displayColor = [(0.8, 1, 0)]\
    }\
}\
\
```\
\
If you paste the example into a .usda file and view the composed\
result in **usdview**, you will see that /TreeA and\
/TreeB both inherit the child prims and properties nested inside\
/\_class\_Tree You can also observe an important property of _inherits_\
behavior in /TreeB/Leaves, which is that inherited opinions are\
weaker than [direct opinions](https://openusd.org/release/glossary.html#usdglossary-directopinion) on prims that\
inherit the opinions. This allows us to always create exceptions to the\
broadcast behavior of inherits, in the same [layerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack) in which we are broadcasting inherited overrides.\
\
The [specifier](https://openusd.org/release/glossary.html#usdglossary-specifier) for /\_class\_Tree is [class](https://openusd.org/release/glossary.html#usdglossary-class). This is not a requirement. A prim can inherit from\
any prim that is neither a descendant nor ancestor of itself, regardless of the\
prim’s specifier or type. However, when inherits are used as a “broadcast edit”\
facilitator, we don’t typically expect the prims into which we deposit the edits\
to be processed directly when we (for example) render a scene - their purpose is\
to convey edits to other prims, and may not even contain a complete definition\
of any prim(s). Using a _class_ specifier for these “edit holders” ensures that\
standard stage traversals will skip the prims, and generally conveys intent that\
the prim will be inherited by other prim(s).\
\
A good way to understand inherits is to start by understanding [references](https://openusd.org/release/glossary.html#usdglossary-references). In the above example, if you replace both\
“inherits = “ with “references = “ and view the composition,\
the results will be indistinguishable from each other! Within a [layerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack) (and ignoring any interaction with [variantSets](https://openusd.org/release/glossary.html#usdglossary-variantset) since VariantSets come between Inherits and\
References in [LIVERPS](https://openusd.org/release/glossary.html#usdglossary-livrpsstrengthordering)) inherits are\
indistinguishable in effect from _local_ references. **The key difference**\
**between references and inherits** is that references fully encapsulate their\
targets, and therefore “disappear” when composed through another layer of\
referencing, whereas the relationship between inheritors and their inherits\
target remains “live” through arbitrary levels of referencing. In other words,\
when a prim _inherits_ from another prim, it **subscribes itself and all**\
**referencing contexts to changes made to the inherited prim.** You can see this\
difference with the following example that uses the previous example as\
Trees.usd:\
\
Forest.usd, demonstrating inherits propagation through references [](https://openusd.org/release/glossary.html#id325 "Permalink to this code")\
\
```\
#usda 1.0\
\
# A new prim, of the same name as the original inherits target, providing new overrides\
class "_class_Tree"\
{\
    token size = "small"\
\
    # It's autumn in California\
    over "Leaves"\
    {\
        color3f[] primvars:displayColor = [(1.0, .1, .1)]\
    }\
}\
\
# TreeB_1 still inherits from _class_Tree because its referent does\
def "TreeB_1" (\
    references = @./Trees.usd@</TreeB>\
)\
{\
}\
\
```\
\
Viewing the [flattened](https://openusd.org/release/glossary.html#usdglossary-flatten) Forest.usd\
you can observe that /TreeB\_1 has both all the\
structure inherited from /\_class\_Tree in Trees.usd, but\
also the size attribute it inherits from /\_class\_Tree in its\
own defining layer; as well, even though the referenced /TreeB had\
specified its own primvars:displayColor for its Leaves prim, the\
reddish color override in /class\_Tree wins. Were you to change the\
_inherits_ to _references_ in Trees.usd, /TreeB\_1 would\
not compose the size attribute and its Leaves would retain their\
original color, and the only way to broadcast changes to all instances of the\
original /\_class\_Tree in Forest.usd would be to\
destructively edit the Trees.usd file. There is a runtime cost to\
keeping inherits live, so you may want to avoid proactively adding inherits\
everywhere just in case you may want to “override all XXX”. Deploy inherits\
where they are likely to be useful; for example, at [asset](https://openusd.org/release/glossary.html#usdglossary-asset) root-prims.\
\
## [Instanceable](https://openusd.org/release/glossary.html\#id395) [](https://openusd.org/release/glossary.html\#instanceable "Permalink to this heading")\
\
_Instanceable_ is a [metadatum](https://openusd.org/release/glossary.html#usdglossary-metadata) that declares\
whether a given [prim](https://openusd.org/release/glossary.html#usdglossary-prim) should be considered as a\
**candidate** for instancing, and is authored via\
[UsdPrim::SetInstanceable](https://openusd.org/release/api/class_usd_prim.html#ad791f3246d4e5bf72e036df7495ec94d). If instanceable = true on a prim,\
then the prim will become an instance of an implicit prototype when composed\
on a [stage](https://openusd.org/release/glossary.html#usdglossary-stage), if and only if the prim also contains one\
or more direct [composition arcs.](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) It does\
not matter whether instanceable is authored in a referenced layer (on\
the prim being referenced) or in the layer (or a super-layer) in which the\
reference is authored: only the [composed value](https://openusd.org/release/glossary.html#usdglossary-valueresolution) on the prim matters. See [Instancing](https://openusd.org/release/glossary.html#usdglossary-instancing) for more information.\
\
## [Instancing](https://openusd.org/release/glossary.html\#id396) [](https://openusd.org/release/glossary.html\#usdglossary-instancing "Permalink to this heading")\
\
_Instancing_ in USD is a feature that allows many instances of “the same” object\
to share the same representation (composed prims) on a UsdStage. In\
exchange for this sharing of representation (which provides speed and memory\
benefits both for the USD core and, generally, for clients processing the\
UsdStage), we give up the ability to uniquely override opinions on prims\
beneath the “instance root”, although it is possible to override opinions that\
will affect **all** instances’ views of the data. Instancing is controlled by\
authored metadata, and can be overridden in stronger layers, so it is possible\
to “break” an instance when necessary, if it must be uniquified.\
\
**Background:**\
\
When you add a [reference](https://openusd.org/release/glossary.html#usdglossary-references), [inherits](https://openusd.org/release/glossary.html#usdglossary-inherits), [specializes](https://openusd.org/release/glossary.html#usdglossary-specializes), or\
[payload](https://openusd.org/release/glossary.html#usdglossary-payload) to a prim, the targeted scene description will\
be composed onto the referencing stage, causing new prims to compose beneath the\
anchoring prim, and allowing the referencing stage to override any of the\
targeted prims or properties. See, for example, the [Trees.usd snippet in the\\
Inherits entry](https://openusd.org/release/glossary.html#usdglossary-inherits). Often we build large environments by\
referencing in many copies of “simple” [assets](https://openusd.org/release/glossary.html#usdglossary-asset) into a\
larger [assembly](https://openusd.org/release/glossary.html#usdglossary-assembly); we add quotes around “simple”\
because it is a matter of perspective and scale: an office chair asset may\
consist of hundreds of prims, for example. Although the asset files are shared\
by a UsdStage each time we add a new reference to any given asset, we\
[compose](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) a unique copy of all of the prims that\
asset contains. This is a requirement to be able to non-destructively edit the\
prims, and conversely for clients to see the unique overrides that may be\
applied to each copy of the asset. However, since number of prims on a stage is\
one of the primary factors that governs how USD scales, this cost can become\
prohibitive as environment size grows, regardless of how many improvements we\
make over time to the per-prim cost in USD.\
\
**Pay for what you need:**\
\
Often, however, an environment will need to express very few overrides on the\
assets it references, and the majority of the overrides it tends to need to\
override bind naturally on the root prim of the asset. This observation provides\
us with a means to apply a philosophy to which we try to adhere broadly in USD:\
pay runtime cost only for the features you need to use. By making a reference\
[instanceable](https://openusd.org/release/glossary.html#usdglossary-instanceable), we declare to USD that we will not\
need to express any overrides on the prims beneath the reference anchor (and any\
overrides already present in the referencing context will be ignored). In\
response to finding an instanceable composition arc, a UsdStage will\
prune its prim composition at the instanceable prim, and make note of the\
targeted layer(s) and the arcs used to target them, as an “instancing key”. The\
stage will create a _prototype_ for each unique instancing key, composing fully -\
_just once_ \- all of the prims that would otherwise appear redundantly under\
each of the instances, and note the relationship between each instance and its\
prototype. Default stage traversals terminate at instances (because instances are\
not allowed to have prim children), and from any prim for which\
[UsdPrim::IsInstance](https://openusd.org/release/api/class_usd_prim.html#a802acbcafc0625377d483931ec4fb202) is true, a client can identify and process its\
prototype using [UsdPrim::GetPrototype](https://openusd.org/release/api/class_usd_prim.html#a5589fb9930ddf2f0ee87f9e6cde87dff).\
\
This behavior can be described as **explicit instances, with implicit**\
**prototypes:** clients are required to be **explicit** about what prims should be\
instanced, so that it is not possible to inadvertently defeat instancing by\
sublayering a new layer that (unintentionally) contains overrides beneath an\
instanced prim in namespace, and so that we can very efficiently determine where\
to apply instancing. Unlike “explicit prototype” schemes, which require (in USD\
terminology) a prim/tree to be explicitly added on a stage as concrete prims\
before adding instances using relationships, a UsdStage manages the\
creation of prototypes for you, as an **implicit result** of which instanceable\
layers you have referenced; this can lead to greater sharing than “explicit\
prototypes” because if instances of the same asset appear in more than one\
referenced assembly in a scene, they will be identified as sharing the same\
prototype, which is not true for explicit prototypes. Extending the\
Trees.usd example, by making both /TreeA and\
/TreeB instanceable, they will share the same composed Trunk\
and Leaves prims inside the prototype created out of\
/\_class\_Tree. **Note that the override expressed on**/TreeB/Leaves **will be ignored**, because we have declared\
/TreeB to be instanceable. Like most features in USD, instanceability\
can be overridden in stronger layers, so if a Forest.usd layer\
referenced /TreeB in Trees.usd and overrode\
instanceable = false, then in that context, /TreeB would get\
back its own Trunk and Leaves children, with the override for\
displayColor on its Leaves child prim.\
\
Instanceable Trees [](https://openusd.org/release/glossary.html#id326 "Permalink to this code")\
\
```\
#usda 1.0\
\
class Xform "_class_Tree"\
{\
    def Mesh "Trunk"\
    {\
        color3f[] primvars:displayColor = [(.8, .8, .2)]\
    }\
\
    def Mesh "Leaves"\
    {\
        color3f[] primvars:displayColor = [(0, 1, 0)]\
    }\
}\
\
def "TreeA" (\
    inherits = </_class_Tree>\
    instanceable = true\
)\
{\
}\
\
def "TreeB" (\
    inherits = </_class_Tree>\
    instanceable = true\
)\
{\
    over "Leaves"\
    {\
        color3f[] primvars:displayColor = [(0.8, 1, 0)]\
    }\
}\
\
```\
\
For more information on usage and examples, see [Scenegraph Instancing](https://openusd.org/release/api/_usd__page__scenegraph_instancing.html) in the USD Manual.\
\
## [Interpolation](https://openusd.org/release/glossary.html\#id397) [](https://openusd.org/release/glossary.html\#interpolation "Permalink to this heading")\
\
_Interpolation_ appears in two different contexts in USD:\
\
- **Temporal Interpolation** of values in attribute value resolution.\
\
By default, when [UsdAttribute::Get](https://openusd.org/release/api/class_usd_attribute.html#a9d41bc223be86408ba7d7f74df7c35a9) [resolves a\\
value from timeSamples](https://openusd.org/release/glossary.html#usdglossary-valueresolution), if the value-type\
supports linear interpolation, the returned value will be linearly interpolated\
between the timeSamples that bracket the requested sample time. This behavior\
can be inhibited for all attributes on a stage by calling\
[UsdStage::SetInterpolationType(UsdInterpolationTypeHeld)](https://openusd.org/release/api/class_usd_stage.html#ad29a9aaba12c36407936a21abf514ea4), which will force all timeSamples to resolve\
with held interpolation.\
\
Linear interpolation is the default interpolation mode for timeSamples because\
composition arcs can apply time-scales and offsets to the data they reference,\
and therefore data that was originally smoothly sampled can easily become poorly\
filtered and sampled in a referencing context if value resolution preformed only\
point or held interpolation: it would become every client’s responsibility to\
attempt to sample the data smoothly, which would be difficult given that the\
function that maps stage-time to the time of the layer in which the timeSamples\
were authored is not easily accessible.\
\
- **Spatial Interpolation** of [Primvar](https://openusd.org/release/glossary.html#usdglossary-primvar) values across a\
[gprim](https://openusd.org/release/glossary.html#usdglossary-gprim).\
\
Interpolation is also the name we give to the metadatum that describes how the\
value(s) in a primvar will interpolate over a [geometric primitive](https://openusd.org/release/glossary.html#usdglossary-gprim) when that primitive is subdivided for rendering. The\
interpretation of interpolation depends on the type of gprim; for example, on a\
Mesh primitive, a primitive can contain a single value to be held across\
the entire mesh, one value per-face, one value per-point to be interpolated\
either linearly or with the mesh’s subdivision basis function, or one value per\
face-vertex. For more information, see [Primvars](https://openusd.org/release/user_guides/primvars.html#primvars).\
\
\
## [IsA Schema](https://openusd.org/release/glossary.html\#id398) [](https://openusd.org/release/glossary.html\#isa-schema "Permalink to this heading")\
\
An _IsA schema_ is a prim [Schema](https://openusd.org/release/glossary.html#usdglossary-schema) that defines the\
prim’s role or purpose on the Stage. The IsA schema to which a prim subscribes\
is determined by the authored value of its [typeName](https://openusd.org/release/api/class_usd_prim.html#aa1b255ef7b406cb036fb6c1b71675b52) metadata, from which it follows that a prim can\
subscribe to at most **one IsA schema** \- unlike [API schemas](https://openusd.org/release/glossary.html#usdglossary-apischema), to which a prim can subscribe many. In terms of the\
USD object model, IsA schemas derive from the C++ class [UsdTyped](https://openusd.org/release/api/class_usd_typed.html) and\
derive their name from the fact that [UsdPrim::IsA<SomeSchemaClass>()](https://openusd.org/release/api/class_usd_prim.html#a6cc1b146723c266ae69fb873bcb2df67) will return true for any prim whose _typeName_ is or derives\
from SomeSchemaClass.\
\
IsA schemas can be either abstract or concrete; [UsdGeomImageable](https://openusd.org/release/api/class_usd_geom_imageable.html) is an\
abstract IsA schema: many prims will answer true to\
UsdPrim::IsA<UsdGeomImageable>(), but there is no\
UsdGeomImageable::Define() method because you cannot create a prim of\
type Imageable. [UsdGeomMesh](https://openusd.org/release/api/class_usd_geom_mesh.html), however, is a concrete IsA\
schema, since it has a Define() method and prims can possess the typeName\
Mesh.\
\
IsA schemas can provide [fallback](https://openusd.org/release/glossary.html#usdglossary-fallback) values for the\
properties they define, which will be reflected at runtime in the [prim\\
definition](https://openusd.org/release/glossary.html#usdglossary-primdefinition).\
\
IsA schemas can be generated using the [USD schema generation tools](https://openusd.org/release/api/_usd__page__generating_schemas.html), but they can also be created\
manually.\
\
## [Kind](https://openusd.org/release/glossary.html\#id399) [](https://openusd.org/release/glossary.html\#usdglossary-kind "Permalink to this heading")\
\
_Kind_ is a reserved, prim-level [metadatum](https://openusd.org/release/glossary.html#usdglossary-metadata) whose\
authored value is a simple string token, but whose interpretation is backed by\
an extensible hierarchical typing-system managed by the [KindRegistry](https://openusd.org/release/api/class_kind_registry.html)\
singleton. We use _kind_ to **classify** [prims](https://openusd.org/release/glossary.html#usdglossary-prim) in USD\
into a higher-level categorization than the prim’s [schema type](https://openusd.org/release/glossary.html#usdglossary-isaschema) provides, principally to assign roles according to\
USD’s organizational notion of “ [Model Hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy)”. Out of the box, USD’s [KindRegistry](https://openusd.org/release/api/class_kind_registry.html)\
comes pre-loaded with a type hierarchy rooted with the base type of “model”, as\
well as an independent type “subcomponent”, like so:\
\
> - [model](https://openusd.org/release/glossary.html#usdglossary-model) \- base class for all model kinds. **“model”**\
> **is considered an abstract type and should not be assigned as any prim’s**\
> **kind.**\
>\
> - [group](https://openusd.org/release/glossary.html#usdglossary-group) \- models that simply group other\
> models. See [Model Hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy) for why we\
> require “namespace contiguity” of models\
>\
> - [assembly](https://openusd.org/release/glossary.html#usdglossary-assembly) \- an important group model, often a\
> published asset or reference to a published asset\
>\
> - [component](https://openusd.org/release/glossary.html#usdglossary-component) \- a “leaf model” that can contain no\
> other models\
>\
> - [subcomponent](https://openusd.org/release/glossary.html#usdglossary-subcomponent) \- an identified, important\
> “sub part” of a component model.\
\
You can query a prim’s kind using [UsdModelAPI::GetKind](https://openusd.org/release/api/class_usd_model_a_p_i.html#a3c54c7584441d0f92eeb6893068f92d2);\
several other queries on UsdPrim are derived from a prim’s kind, such as the\
[UsdPrim::IsModel](https://openusd.org/release/api/class_usd_prim.html#ad93a1d74f88e0a32113df97b3358e9d3) and [UsdPrim::IsGroup](https://openusd.org/release/api/class_usd_prim.html#ad534b1a75ec6ebe6cb3d5883d683ec93) queries.\
\
## [Layer](https://openusd.org/release/glossary.html\#id400) [](https://openusd.org/release/glossary.html\#usdglossary-layer "Permalink to this heading")\
\
A _Layer_ is the atomic persistent container of scene description for USD. A\
layer contains zero or more [PrimSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec), that in turn\
describe [Property](https://openusd.org/release/glossary.html#usdglossary-property) and [Metadata](https://openusd.org/release/glossary.html#usdglossary-metadata) values. Each layer possesses an _identifier_ that can\
be used to construct references to the layer from other layers.\
\
[SdfLayer](https://openusd.org/release/api/class_sdf_layer.html) provides both the “document model” for layers, and the\
interface by which USD authors to and extracts data from layers. The\
[SdfLayer](https://openusd.org/release/api/class_sdf_layer.html) interface serves data according to the USD\
prim/property/metadata data model, but the actual encoding of data in the\
backing file is quite flexible, thanks to the [SdfFileFormat](https://openusd.org/release/api/class_sdf_file_format.html) plugin\
interface. By implementing a sub-class of [SdfFileFormat](https://openusd.org/release/api/class_sdf_file_format.html) and\
associating it with a unique file extension for USD’s consumption, we can enable\
direct USD referencing of layers expressed as files of any format whose encoding\
can reasonably be translated into USD. This is not only how USD supports direct\
consumption of Alembic (.abc) files, but also how USD’s native text\
and [crate binary](https://openusd.org/release/glossary.html#usdglossary-cratefileformat) representations are\
provisioned.\
\
Data authored to layers by applications or scripts **will remain in memory**\
**until** [SdfLayer::Save](https://openusd.org/release/api/class_sdf_layer.html#ad5a54175a9d67b72094f8fdf3e83c9d9) **is called on the layer**. If a program is\
writing more data than fits in the program’s memory allotment, we suggest:\
\
> 1. Using USD’s native binary [crate format](https://openusd.org/release/glossary.html#usdglossary-cratefileformat)\
> (which is the default file format for files created with the\
> .usd file extension)\
>\
> 2. Calling layer.Save() periodically: doing so will flush all of\
> the heavy property-value data from memory into the file, while leaving\
> the file open and available for continued writing of data. Only the crate\
> binary format possesses this “flushability” property; USD’s text\
> representation can only be written out sequentially beginning-to-end, and\
> cannot be digested lazily, therefore it cannot be authored incrementally\
> and must always keep all of its data in-memory; other formats do not\
> allow incremental saving because they must translate USD’s encoding into\
> their own format that does not itself allow incremental saving, like the\
> [Alembic FileFormatPlugin](https://openusd.org/release/plugins_alembic.html).\
\
Although layers exist first and foremost to define the persistent storage\
representation of USD data, one can also create temporary, “in-memory” layers\
for lightweight (in that there is no filesystem access required) USD data\
storage, via [UsdStage::CreateInMemory](https://openusd.org/release/api/class_usd_stage.html#a7fabfeea7271b1e6e5f308865f146391).\
\
## [Layer Offset](https://openusd.org/release/glossary.html\#id401) [](https://openusd.org/release/glossary.html\#layer-offset "Permalink to this heading")\
\
[Composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) such as [References](https://openusd.org/release/glossary.html#usdglossary-references) and [SubLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers) can\
include an offset and scaling of time to be applied during attribute [value\\
resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution) for all data composed from the\
target layer. We call this a _Layer Offset_, embodied in\
[SdfLayerOffset](https://openusd.org/release/api/class_sdf_layer_offset.html). Layer offsets compose, so that if **A** references\
**B** with a time-scale of 2.0 and **B** references **C** with a time-scale of\
3.5, then data resolved at **A** whose source is **C** will have a total\
time-scale of 7.0 applied to it.\
\
When an arc has both an offset and scale applied, the referenced animation is\
**first scaled, then offset** as it is brought into the referencing layer. So,\
in the following example, a [timeSample](https://openusd.org/release/glossary.html#usdglossary-timesample) at [timeCode](https://openusd.org/release/glossary.html#usdglossary-timecode) 12 in the file someAnimation.usd will\
appear at ((12 \* 0.5) + 10) = **16** as resolved from the referencing\
layer. Layer offsets cannot themselves vary over time. If a consuming context\
requires variable retiming of referenced data, it can use the more powerful (and\
somewhat more costly) [Value Clips](https://openusd.org/release/glossary.html#usdglossary-valueclips) feature.\
\
SubLayer offset/scale in usda [](https://openusd.org/release/glossary.html#id327 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    subLayers = [\
        @./someAnimation.usd@ (offset = 10; scale = 0.5)\
    ]\
)\
\
```\
\
## [LayerStack](https://openusd.org/release/glossary.html\#id402) [](https://openusd.org/release/glossary.html\#usdglossary-layerstack "Permalink to this heading")\
\
_LayerStacks_ are the keystone to understanding [composition](https://openusd.org/release/glossary.html#usdglossary-composition) in USD. The **definition** of a LayerStack is\
simply:\
\
> - **LayerStack:** The ordered set of layers resulting from the recursive\
> gathering of all [SubLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers) of a [Layer](https://openusd.org/release/glossary.html#usdglossary-layer), plus the layer itself as first and strongest.\
\
The LayerStack is important to understanding composition for two reasons:\
\
> 1. **Composition Arcs target LayerStacks, not Layers.**\
>\
> When a layer [references](https://openusd.org/release/glossary.html#usdglossary-references) (or [payloads](https://openusd.org/release/glossary.html#usdglossary-payload) or sub-layers) another layer, it is targeting\
> (and therefore composing) not just the data in the single layer, but all\
> the data (in strength-order) in the _LayerStack_ rooted at the targeted\
> layer.\
>\
> 2. **LayerStacks provide the container through which references can be**\
> **list-edited.**\
>\
> Many of the composition arcs (as well as [relationships](https://openusd.org/release/glossary.html#usdglossary-relationship)) describe not just a single target, but an\
> _orderable list of targets_, that will be processed (in order) according\
> to the type of the arc. References can be [list edited](https://openusd.org/release/glossary.html#usdglossary-listediting) among the layers of a LayerStack. This can\
> be a powerful method of non-destructively changing the large-scale\
> structure of a scene as it flows down the pipeline.\
\
For example, we might have a generic version of a special effect added into a\
scene at the sequence level:\
\
sequenceFX.usd, which adds a reference onto an asset that may be\
referenced in from a weaker layer [](https://openusd.org/release/glossary.html#id328 "Permalink to this code")\
\
```\
#usda 1.0\
\
over "World"\
{\
    over "Props"\
    {\
        over "Prop_145" (\
            prepend references = @sequenceFX/turbulence.usd@\
        )\
        {\
        }\
    }\
}\
\
```\
\
Now, at the shot-level, we have a shotFX.usd layer that participates\
in the same LayerStack as sequenceFX.usd (because one of the shot\
layers SubLayers in the sequence.usd layer, which in turn SubLayers\
the above sequenceFX.usd layer). In this particular shot, we need to\
replace the generic turbulence effect with a different one, which may have\
completely different prims in it. Therefore it is not enough to just “add on” an\
extra reference, because the prims from turbulence.usd will still\
“shine through” - we must also **remove** the weaker reference, which we can do\
via list editing.\
\
shotFX.usd, which replaces the sequence-level reference, while\
preserving any other references [](https://openusd.org/release/glossary.html#id329 "Permalink to this code")\
\
```\
#usda 1.0\
\
over "World"\
{\
    over "Props"\
    {\
        over "Prop_145" (\
            prepend references = @./fx/shotEffect1.usd@\
            delete references = @sequenceFX/turbulence.usd@\
        )\
        {\
        }\
    }\
}\
\
```\
\
When the shot is composed, the references on /World/Props/Prop\_145\
will be combined using the list editing rules, and will resolve to a list that\
includes shotEffect1.usd, but **not** turbulence.usd. In\
this second example we have also shown that the operand of list-editing\
operations can be a list that can contain multiple targets.\
\
## [List Editing](https://openusd.org/release/glossary.html\#id403) [](https://openusd.org/release/glossary.html\#list-editing "Permalink to this heading")\
\
_List editing_ is a feature to which some array-valued data elements in USD can\
subscribe, that allows the array-type element to be non-destructively, sparsely\
modified across [composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) . It would\
be very expensive and difficult to reason about list editable elements that are\
also time-varying, so [Attributes](https://openusd.org/release/glossary.html#usdglossary-attribute) can never be list\
editable. [Relationships](https://openusd.org/release/glossary.html#usdglossary-relationship), [References](https://openusd.org/release/glossary.html#usdglossary-references), [Inherits](https://openusd.org/release/glossary.html#usdglossary-inherits),\
[Specializes](https://openusd.org/release/glossary.html#usdglossary-specializes), [VariantSets](https://openusd.org/release/glossary.html#usdglossary-variantset), and integer and string/token array [custom\\
metadata](https://openusd.org/release/glossary.html#usdglossary-metadata) can be list edited. When an element is list\
editable, instead of only being able to assign an explicit value to it, you can\
also, in any stronger layer:\
\
> - **append** another value or values **to the back** of the resolved list;\
> if the values already appear in the resolved list, they will be reshuffled\
> to the back. An appended composition arc in a **stronger** layer of a\
> LayerStack will therefore be **weaker** than all of the arcs _of the same_\
> _type_ appended from weaker layers, by default; however, the Usd API’s for\
> adding composition arcs give you some flexibility here.\
>\
> - **delete** a value or values from the resolved list. A “delete” statement\
> can be speculative, that is, it is not an error to attempt to delete a\
> value that is not present in the resolved list.\
>\
> - **prepend** another value or values **on the front** of the resolved list;\
> if the values already appear in the resolved list, they will be shuffled\
> to the front. A prepended composition arc in a **weaker** layer of a\
> LayerStack will still be **stronger** than any arcs _of the same type_\
> that are **appended** from **stronger** layers.\
>\
> - **reset to explicit** , which is an “unqualified” operation, as in\
> references = @myFile.usd@. This causes the resolved list to be reset\
> to the provided value or values, **ignoring all list ops from weaker**\
> **layers**.\
\
Although we refer to these operations as “list editing”, and they operate on\
array-valued data, it should be clear from the description of the operators that\
list-edited elements _always resolve to a set_ , that is, there will never be\
any repetition of values in the resolved list, and it is a syntax error for the\
same value to appear twice in the same operation in a layer. Also, in the usda\
text syntax, any operation can assign either a single value _without_ the\
square-bracket list delimiters, or a sequence of values inside square brackets.\
\
See [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack) for an example of list editing, as\
applied to references. See also the FAQ on deleting items with list ops:\
[When can you delete a reference (or other deletable thing)?](https://openusd.org/release/usdfaq.html#when-can-you-delete-a-reference-or-other-deletable-thing)\
\
## [LIVERPS Strength Ordering](https://openusd.org/release/glossary.html\#id404) [](https://openusd.org/release/glossary.html\#liverps-strength-ordering "Permalink to this heading")\
\
LIVERPS is an acronym for **Local, Inherits, VariantSets, Relocates, References,**\
**Payload, Specializes**, and is the fundamental rubric for understanding how\
[opinions](https://openusd.org/release/glossary.html#usdglossary-opinions) and\
[namespace](https://openusd.org/release/glossary.html#usdglossary-namespace) compose in USD. **LIVERPS** describes\
the strength ordering in which the various composition arcs combine,\
**within each** [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack). For example, when we\
are trying to determine the value of an [attribute](https://openusd.org/release/glossary.html#usdglossary-attribute)\
or [metadatum](https://openusd.org/release/glossary.html#usdglossary-metadata) on a stage at _path_ that subscribes\
to the [value resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution) policy that\
“strongest opinion wins” (which is all attributes and most metadata), we iterate\
through [PrimSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec) in the following order looking\
for an opinion for the requested datum:\
\
> 1. **Local**:\
>\
> Iterate through all the layers in the local LayerStack\
> looking for opinions on the PrimSpec at _path_ in each layer - recall\
> that according to the definition of LayerStack, this is where the effect\
> of direct opinions in all [SubLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers) of the\
> root layer of the LayerStack will be consulted. If no opinion is found,\
> then…\
>\
> 2. **Inherits**:\
>\
> Resolve the [Inherits](https://openusd.org/release/glossary.html#usdglossary-inherits) affecting\
> the prim at _path_, and iterate through the resulting targets. For\
> each target, **recursively apply** **LIVERP** **evaluation** on\
> the targeted LayerStack - **Note that the “S” is not present** \- we\
> ignore Specializes arcs while recursing . If no opinion is found,\
> then…\
>\
> 3. **VariantSets**:\
>\
> Apply the resolved variant selections to all\
> [VariantSets](https://openusd.org/release/glossary.html#usdglossary-variantset) that affect the PrimSpec at\
> _path_ in the LayerStack, and iterate through the selected\
> [Variants](https://openusd.org/release/glossary.html#usdglossary-variant) on each VariantSet. For each\
> target, **recursively apply** **LIVERP** **evaluation** on the targeted\
> LayerStack - **Note that the “S” is not present** \- we ignore Specializes\
> arcs while recursing. If no opinion is found, then…\
>\
> 4. **r(E)locates**:\
>\
> Resolve any [Relocates](https://openusd.org/release/glossary.html#usdglossary-relocates) target path\
> affecting the prim at _path_, and iterate through the resulting relocates\
> source. For each source, **recursively apply** **LIVERP** **evaluation**\
> on the source’s remote LayerStack - **Note that the “S” is not present**\
> \- we ignore Specializes arcs while recursing. If no opinion is found,\
> then…\
>\
> 5. **References**:\
>\
> Resolve the [References](https://openusd.org/release/glossary.html#usdglossary-references)\
> affecting the prim at _path_, and iterate through the resulting\
> targets. For each target, **recursively apply** **LIVERP** **evaluation**\
> on the targeted LayerStack - **Note that the “S” is not present** \- we\
> ignore Specializes arcs while recursing. If no opinion is found, then…\
>\
> 6. **Payload**:\
>\
> Resolve the [Payload](https://openusd.org/release/glossary.html#usdglossary-payload)\
> arcs affecting the prim at _path_; if _path_ has been **loaded on**\
> **the stage,** iterate through the resulting targets just as we would\
> references from step 4. If no opinion is found, then…\
>\
> 7. **Specializes**:\
>\
> Resolve the [Specializes](https://openusd.org/release/glossary.html#usdglossary-specializes)\
> affecting the prim at _path_, and iterate through the resulting\
> targets, **recursively applying \*full\* LIVERPS evaluation** on each target\
> prim. If no opinion is found, then…\
>\
> 8. Indicate that we could find no authored opinion\
\
We have omitted some details, such as how, for any composition arc in the above\
recipe, we order arcs applied directly on the PrimSpec in relation to the same\
kind of arc authored on an _ancestral_ PrimSpec in the LayerStack - the short\
answer is that “ancestral arcs” are weaker than “direct arcs” (see\
[Direct Opinion](https://openusd.org/release/glossary.html#usdglossary-directopinion)). Additionally, we skip over “S” during\
recursion for other arcs, as explained further in the entry for\
[Specializes](https://openusd.org/release/glossary.html#usdglossary-specializes). Performing every step as described\
above for each value lookup would be very costly. This is why we compute and\
cache an [Index](https://openusd.org/release/glossary.html#usdglossary-index) for every prim on the Stage. The Index\
“pre-applies” the full algorithm to gather all PrimSpecs contributing opinions\
to a prim, caching the list in a recursive data structure that allows efficient\
processing whenever a value on the prim needs to be resolved.\
\
The algorithm for computing the namespace of the stage (i.e. what prims are\
present and where) are slightly more involved, but still follows the LIVERPS\
recipe.\
\
## [Load / Unload](https://openusd.org/release/glossary.html\#id405) [](https://openusd.org/release/glossary.html\#load-unload "Permalink to this heading")\
\
The [Payload arc](https://openusd.org/release/glossary.html#usdglossary-payload) provides a “deferred reference”\
behavior. Wherever a Stage contains at least one Payload (payloads can be\
list-edited and chained), the client has the ability to **Load** (compose) all\
the scene description targeted by the Payload, or to **Unload** the Payloads and\
all their scene description, recomposing all prims beneath the payloaded-prim,\
recursively unloading _their_ payloads, should they possess any. We generally\
associate payloads with “model assets”, which provides us with payloads, and\
therefore “load points” at all the leaves of the [Model Hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy). This organization allows clients to craft\
“working sets” of a Stage, fully composing only the parts of the scene needed\
for a given task, saving time and memory for the operation.\
\
For more information, see [Working Set Management](https://openusd.org/release/api/class_usd_stage.html#Usd_workingSetManagement) in the USD Manual.\
\
## [Localize](https://openusd.org/release/glossary.html\#id406) [](https://openusd.org/release/glossary.html\#localize "Permalink to this heading")\
\
USD allows the construction of highly referenced and layered scenes that\
assemble files from many different sources, which may resolve differently in\
different contexts (for example, your [asset resolver](https://openusd.org/release/glossary.html#usdglossary-assetresolution) may apply external state to select between\
multiple versions of an asset). If one wishes to package up all of the needed\
files for a given scene so that they are isolated from asset resolution behavior\
and can be copied or shipped easily to another location, **without** the drastic\
transformation that [flattening a stage](https://openusd.org/release/glossary.html#usdglossary-flatten) incurs, then\
one must “localize” all of the scattered layers into a coherent tree of\
files, which requires not only copying files, but also editing them to retarget\
all of the [references](https://openusd.org/release/glossary.html#usdglossary-references), [payloads](https://openusd.org/release/glossary.html#usdglossary-payload), and generic asset paths to target the copied files in\
their new locations. [UsdUtilsCreateNewUsdzPackage](https://openusd.org/release/api/usdz_package_8h.html#a30233ff311212434bc7f7a92a9ce06f9) does this for you,\
although we have not yet exposed the ability to _just_ localize yet, but we hope\
to eventually.\
\
## [Metadata](https://openusd.org/release/glossary.html\#id407) [](https://openusd.org/release/glossary.html\#usdglossary-metadata "Permalink to this heading")\
\
_Metadata_ is the lightest-weight form of (name, value) scene description; it is\
“light” because unlike attributes, metadata cannot be time-varying, and because\
prims and properties can possess metadata, but metadata cannot itself have\
metadata. Metadata are extensible, however adding a new, named piece of\
metadata requires a change to a configuration file to do so, because the\
software wants to know, definitively, what the datatype of the metadatum should\
be. USD provides a special, dictionary-valued metadatum called customData\
that provides a place to put user metadata without needing to touch any\
configuration files. For more information on the allowed types for metadata and\
how to add new metadata to the system, please see [the discussion of metadata in\\
the API manual.](https://openusd.org/release/api/_usd__page__object_model.html#Usd_OM_Metadata)\
\
## [Model](https://openusd.org/release/glossary.html\#id408) [](https://openusd.org/release/glossary.html\#usdglossary-model "Permalink to this heading")\
\
_Model_ is a scenegraph annotation ascribable to [prims](https://openusd.org/release/glossary.html#usdglossary-prim) by\
setting their [kind](https://openusd.org/release/glossary.html#usdglossary-kind) metadata.\
We label certain prims as models to partition large scenegraphs into\
more manageable pieces - there is a core “leaf model” kind, [component](https://openusd.org/release/glossary.html#usdglossary-component), and two refinements of “models that aggregate\
other models”, [group](https://openusd.org/release/glossary.html#usdglossary-group), and [assembly](https://openusd.org/release/glossary.html#usdglossary-assembly). Core [UsdPrim](https://openusd.org/release/api/class_usd_prim.html) API can cheaply answer\
questions like [UsdPrim::IsModel](https://openusd.org/release/api/class_usd_prim.html#ad93a1d74f88e0a32113df97b3358e9d3) and [UsdPrim::IsGroup](https://openusd.org/release/api/class_usd_prim.html#ad534b1a75ec6ebe6cb3d5883d683ec93), and\
“model-ness” is one of the criteria that a [UsdPrimRange](https://openusd.org/release/api/class_usd_prim_range.html) can use to\
traverse a stage. All model kinds are [extensible via site-customization](https://openusd.org/release/api/kind_page_front.html#kind_extensions), so that, for example, you can have\
both “character” and “prop” component model kinds in your pipeline if that is a\
useful distinction to make. See also [Model Hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy).\
\
## [Model Hierarchy](https://openusd.org/release/glossary.html\#id409) [](https://openusd.org/release/glossary.html\#usdglossary-modelhierarchy "Permalink to this heading")\
\
_Model Hierarchy_ builds on the concept of [model](https://openusd.org/release/glossary.html#usdglossary-model) in a\
scenegraph to tackle the problem of discovering and defining a “table of\
contents of important subtrees of prims” that can be enumerated and traversed\
very efficiently. The model hierarchy defines a **contiguous** set of prims\
descending from a root prim on a stage, **all of which are models**. Model\
hierarchy is an index of the scene that is, strictly, a **prefix**, of the\
entire scene. The member prims must adhere to the following three rules:\
\
> 1. Only [group model prims](https://openusd.org/release/glossary.html#usdglossary-group) can have other model\
> children ( [assembly](https://openusd.org/release/glossary.html#usdglossary-assembly) is a [kind of](https://openusd.org/release/glossary.html#usdglossary-kind) group)\
>\
> 2. A prim can only be a model if its parent prim is also a (group) model -\
> except for the root model prim.\
>\
> 3. No prim should have the exact kind “model”, because “model” is neither a\
> singular component nor a plural group container - it is just the\
> “abstract” commonality between components and groups.\
\
This implies that [component](https://openusd.org/release/glossary.html#usdglossary-component) models cannot have model\
children. It also implies that just because a prim has its [kind](https://openusd.org/release/glossary.html#usdglossary-kind) metadata authored to “component”, its\
[UsdPrim::IsModel](https://openusd.org/release/api/class_usd_prim.html#ad93a1d74f88e0a32113df97b3358e9d3) query will only return true if its parent\
[UsdPrim::IsGroup](https://openusd.org/release/api/class_usd_prim.html#ad534b1a75ec6ebe6cb3d5883d683ec93) also answers affirmatively.\
\
We find model hierarchy to be a useful construct because the models in our\
scenes align very closely with “referenced assets”, and we build complex scenes\
by referencing many assets together and overriding them. Reasoning about\
referencing structure can get complicated very quickly and necessitate\
introducing fragile conventions. However, reasoning about a model hierarchy is\
much more straightforward, and when assets are published/packaged with model\
kinds already established, the model hierarchy becomes mostly “self assembling”.\
\
## [Namespace](https://openusd.org/release/glossary.html\#id410) [](https://openusd.org/release/glossary.html\#usdglossary-namespace "Permalink to this heading")\
\
_Namespace_ is simply the term USD uses to describe the set of prim [paths](https://openusd.org/release/glossary.html#usdglossary-path) that provide the identities for prims on a [Stage](https://openusd.org/release/glossary.html#usdglossary-stage), or [PrimSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec) in a [Layer](https://openusd.org/release/glossary.html#usdglossary-layer). A Stage’s namespace nominally consists of a “forest”\
in graph theory, that is, any number of “root prims” that have (possibly empty)\
trees beneath them. To facilitate traversal and processing of a Stage’s\
namespace of prims, each Stage possesses a [PseudoRoot](https://openusd.org/release/glossary.html#usdglossary-pseudoroot) prim that is the parent of all authored root prims,\
represented by the path /.\
\
## [Opinions](https://openusd.org/release/glossary.html\#id411) [](https://openusd.org/release/glossary.html\#opinions "Permalink to this heading")\
\
_Opinions_ are the atomic elements that participate in [Value Resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution) in USD. Each time you author a value for a\
Metadatum, Attribute, or Relationship, you are expressing an _opinion_ for that\
object in a PrimSpec in a particular Layer. On a composed Stage, any object\
may be affected by multiple opinions from different layers; the ordering of\
these opinions is determined by the [LIVERPS strength ordering](https://openusd.org/release/glossary.html#usdglossary-livrpsstrengthordering).\
\
## [Over](https://openusd.org/release/glossary.html\#id412) [](https://openusd.org/release/glossary.html\#usdglossary-over "Permalink to this heading")\
\
_Over_ is one of the three possible [specifiers](https://openusd.org/release/glossary.html#usdglossary-specifier) a\
[prim](https://openusd.org/release/glossary.html#usdglossary-prim) (and also a [PrimSpec](https://openusd.org/release/glossary.html#usdglossary-primspec))\
can possess. An _over_ is the “weakest” of the three specifiers, in that it\
does not change the resolved specifier of a prim in a LayerStack even when an\
_over_ appears in a stronger layer than a _def_ or _class_ for the same\
PrimSpec. _Over_ is short for “override” or “compose over”, and its purpose is\
just to provide a _speculative_, neutral prim container for overriding\
opinions; we use the term “speculative” because if the over composes over a\
defined prim, its opinions will contribute to the evaluation of the stage, but\
if all the PrimSpecs contributing to a prim have the _over specifier_, then the\
prim will not be visited by [UsdPrim::GetChildren](https://openusd.org/release/api/class_usd_prim.html#a2619563fc9180d8ead597944fd7f6ec7) or\
[UsdStage::Traverse](https://openusd.org/release/api/class_usd_stage.html#adba675b55f41cc1b305bed414fc4f178).\
\
When an application exports sparse overrides into a layer that sits on top of an\
existing composition, it is common to see deep nesting of overs.\
\
An “over” provides speculative opinions [](https://openusd.org/release/glossary.html#id330 "Permalink to this code")\
\
```\
#usda 1.0\
\
over "World"\
{\
    over "Props"\
    {\
        over "LuxoBall"\
        {\
            double radius = 10\
        }\
    }\
}\
\
```\
\
## [Path](https://openusd.org/release/glossary.html\#id413) [](https://openusd.org/release/glossary.html\#usdglossary-path "Permalink to this heading")\
\
A _path_ is a location in [namespace](https://openusd.org/release/glossary.html#usdglossary-namespace). In USD text\
syntax (and documentation), paths are enclosed in angle-brackets, as found in\
the authored targets for [references](https://openusd.org/release/glossary.html#usdglossary-references), [payloads](https://openusd.org/release/glossary.html#usdglossary-payload), [inherits](https://openusd.org/release/glossary.html#usdglossary-inherits), [specializes](https://openusd.org/release/glossary.html#usdglossary-specializes), and [relationships](https://openusd.org/release/glossary.html#usdglossary-relationship).\
USD assigns paths to all elements of scene description other than metadata,\
and the concrete embodiment of a path, [SdfPath](https://openusd.org/release/api/class_sdf_path.html), serves in the API as a\
compact, thread-safe, key by which to fetch and store scene description, both\
within a Layer, and composed on a Stage. The [SdfPath](https://openusd.org/release/api/class_sdf_path.html) syntax allows for\
recording paths to different kinds of scene description. For example:\
\
> 1. /Root/Child/Grandchild represents an absolute prim path of\
> three nested prims\
>\
> 2. /Root/Child/Grandchild.visibility names the property\
> visibility on the prim Grandchild.\
>\
> 3. /Root/Child/Grandchild{modelingVariant=withCargoRack}/GreatGrandchild\
> represents the child prim GreatGrandchild authored _inside_ the\
> [Variant](https://openusd.org/release/glossary.html#usdglossary-variant) “withCargoRack of [VariantSet](https://openusd.org/release/glossary.html#usdglossary-variantset) modelingVariant\
\
Scene description in a Layer corresponding to example paths [](https://openusd.org/release/glossary.html#id331 "Permalink to this code")\
\
```\
#usda 1.0\
\
def "Root"\
{\
    def "Child"\
    {\
        def "GrandChild" (    # Corresponds to path #1 above\
            add variantSets = [ "modelingVariant" ]\
        {\
            variantSet "modelingVariant" = {\
                "withCargoRack" {\
                    def "GreatGrandchild"   # Corresponds to path #3 above\
                    {\
                    }\
                }\
            }\
\
            token visibility   # Corresponds to path #2 above\
        }\
    }\
}\
\
```\
\
## [Path Translation](https://openusd.org/release/glossary.html\#id414) [](https://openusd.org/release/glossary.html\#usdglossary-pathtranslation "Permalink to this heading")\
\
All of USD’s [Composition Arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) other than\
[SubLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers) allow a “prim name change” as the target\
prim of the arc gets composed under the prim that added the composition arc.\
One of a [UsdStage’s](https://openusd.org/release/glossary.html#usdglossary-stage) central responsibilities is applying\
all of the necessary **path translation** required to allow its users to deal\
almost exclusively in the “fully composed namespace” of the stage’s root layer,\
rather than needing to be concerned about what layer (with its own namespace)\
provides the data we want to access or author. Path translation is applied\
during such queries as [finding a prim](https://openusd.org/release/api/class_usd_prim.html#a2619563fc9180d8ead597944fd7f6ec7)\
and [fetching the targets](https://openusd.org/release/api/class_usd_relationship.html#aaa227b81fdab5fe41bcc0ca21133a0f8)\
of a [relationship](https://openusd.org/release/glossary.html#usdglossary-relationship) or [connection](https://openusd.org/release/glossary.html#usdglossary-connection), and _inverse_ path translation is performed by the\
active [Edit Target](https://openusd.org/release/glossary.html#usdglossary-edittarget) whenever you author to a stage.\
\
Following is an example of the kind of path translation that happens in response\
to adding references, demonstrating the effects on not only the stage’s\
namespace, but also on relationship targets. Given:\
\
asset.usd [](https://openusd.org/release/glossary.html#id332 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    defaultPrim = "MyModel"\
)\
\
def Xform "MyModel"\
{\
    rel gprims = [ </MyModel/Cube>, </MyModel/Sphere> ]\
\
    def Cube "Cube"\
    {\
    }\
}\
\
```\
\
and an aggregating file that references asset.usd:\
\
assembly.usd [](https://openusd.org/release/glossary.html#id333 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    defaultPrim = "MySet"\
)\
\
def Xform "MySet"\
{\
    def Xform "Building_1" (\
        references = @asset.usd@\
    )\
    {\
    }\
\
    def Xform "Building_2" (\
        references = @asset.usd@\
    )\
    {\
    }\
}\
\
```\
\
**Then**, if we add the “set” represented by assembly.usd into a\
shot.usd, USD path translation operates at two levels (recursively),\
translating /MyModel to either /Building\_1 or\
/Building\_2, conextually, and translating /MySet to\
/WestVillage. So if we [query the targets](https://openusd.org/release/api/class_usd_relationship.html#aaa227b81fdab5fe41bcc0ca21133a0f8) of the relationship\
/World/WestVillage/Building\_1.gprims, we will get back:\
\
> \[ /World/WestVillage/Building\_1/Cube, /World/WestVillage/Building\_1/Sphere \]\
\
If we note that there _is_ no Sphere prim, and therefore want to\
eliminate it from consideration at the shot-level using relationship\
[list-editing](https://openusd.org/release/glossary.html#usdglossary-listediting), **we refer to it by its stage-level**\
**path** , rather than its authored path, which is not even easy to determine\
using the public Usd API’s. Deleting the target in shot.usd might\
look like this:\
\
shot.usd [](https://openusd.org/release/glossary.html#id334 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    defaultPrim = "World"\
)\
\
def Xform "World"\
{\
    def "WestVillage" (\
        references = @assembly.usd@\
    )\
    {\
        over "Building_1"\
        {\
            delete rel gprims = </World/WestVillage/Building_1/Sphere>\
        }\
    }\
}\
\
```\
\
We mentioned above that path translation also operates in the opposite direction\
when you use Edit Targets to send your relationship or connection edits across a\
composition arc, because it follows that **every encoded path must be in the**\
**namespace of the** [PrimSpec](https://openusd.org/release/glossary.html#usdglossary-primspec) **on which it is**\
**recorded**. For example, if we were working with the same shot.usd\
Stage, and specified the **same path to delete**,\
/World/WestVillage/Building\_1/Sphere, but have set the Stage’s\
EditTarget to assembly.usd across the reference authored on\
/World/WestVillage/Building\_1, then the act of authoring will\
transform the path into the _target_ namespace, so the result would be an\
assembly.usd that looks like:\
\
Modified assembly.usd with relationship deletion [](https://openusd.org/release/glossary.html#id335 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    defaultPrim = "MySet"\
)\
\
def Xform "MySet"\
{\
    def Xform "Building_1" (\
        references = @asset.usd@\
    )\
    {\
        delete rel gprims = </MySet/Building_1/Sphere>\
    }\
\
    def Xform "Building_2" (\
        references = @asset.usd@\
    )\
    {\
    }\
}\
\
```\
\
## [Payload](https://openusd.org/release/glossary.html\#id415) [](https://openusd.org/release/glossary.html\#usdglossary-payload "Permalink to this heading")\
\
A _Payload_ is a [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) that is a\
special kind of a [Reference](https://openusd.org/release/glossary.html#usdglossary-references). It is different from\
references primarily in two ways:\
\
> - The targets of References are always consumed greedily by the [indexing\\
> algorithm](https://openusd.org/release/glossary.html#usdglossary-index) that is used to open and build a\
> Stage. When a Stage is opened with\
> [UsdStage::InitialLoadSet::LoadNone](https://openusd.org/release/api/class_usd_stage.html#a7a661b8f729152c673078fbbafe29a46ae9175fcb3934ccf24235741a50556ab3) specified, Payload\
> arcs are recorded, but not traversed. This behavior allows clients to\
> manually construct a “working set” that is a subset of the whole scene, by\
> [loading](https://openusd.org/release/glossary.html#usdglossary-load-unload) just the bits of the scene that they\
> require.\
>\
> - [Payloads are weaker than references](https://openusd.org/release/glossary.html#usdglossary-livrpsstrengthordering), so, for a particular prim within\
> any given [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack), all direct references\
> will be stronger than all direct payloads.\
\
Although payloads can be authored on any prim in any layer, in Pixar’s pipeline\
we find it very useful to primarily add payloads to the root prims of\
[component-model](https://openusd.org/release/glossary.html#usdglossary-component) assets. See [the performance\\
note on packaging assets with payloads](https://openusd.org/release/maxperf.html#package-assets-with-payloads).\
\
## [Prim](https://openusd.org/release/glossary.html\#id416) [](https://openusd.org/release/glossary.html\#usdglossary-prim "Permalink to this heading")\
\
A _Prim_ is the primary container object in USD: prims can contain (and order)\
other prims, creating a “ [namespace hierarchy](https://openusd.org/release/glossary.html#usdglossary-namespace)” on a\
[Stage](https://openusd.org/release/glossary.html#usdglossary-stage), and prims can also contain (and order)\
[properties](https://openusd.org/release/glossary.html#usdglossary-property) that hold meaningful data. Prims, along\
with their associated, computed [indices](https://openusd.org/release/glossary.html#usdglossary-index), are the only\
persistent scenegraph objects that a Stage retains in memory, and the API for\
interacting with prims is provided by the [UsdPrim](https://openusd.org/release/api/class_usd_prim.html) class. Prims always\
possess a resolved [Specifier](https://openusd.org/release/glossary.html#usdglossary-specifier) that determines the\
prim’s generic role on a stage, and a prim _may_ possess a [schema](https://openusd.org/release/glossary.html#usdglossary-schema) typeName that dictates what kind of data the prim\
contains. Prims also provide the granularity at which we apply [scene-level\\
instancing](https://openusd.org/release/glossary.html#usdglossary-instancing), [load/unload behavior](https://openusd.org/release/glossary.html#usdglossary-load-unload), and [deactivation](https://openusd.org/release/glossary.html#usdglossary-active-inactive).\
\
## [Prim Definition](https://openusd.org/release/glossary.html\#id417) [](https://openusd.org/release/glossary.html\#usdglossary-primdefinition "Permalink to this heading")\
\
A _prim definition_ is the set of built-in [properties](https://openusd.org/release/glossary.html#usdglossary-property)\
and [metadata](https://openusd.org/release/glossary.html#usdglossary-metadata) that a [prim](https://openusd.org/release/glossary.html#usdglossary-prim) gains\
from a combination of the [IsA schema](https://openusd.org/release/glossary.html#usdglossary-isaschema) determined from\
its typeName and its [applied API schemas](https://openusd.org/release/glossary.html#usdglossary-apischema). A prim’s\
prim definition is used to determine what properties and metadata the prim has\
besides what is authored in its scene description. It also may provide [fallback](https://openusd.org/release/glossary.html#usdglossary-fallback) values during property value or metadata value\
resolution for the prim’s built-in properties and metadata. The API for prim\
definitions are provided by the [UsdPrimDefinition](https://openusd.org/release/api/class_usd_prim_definition.html) class.\
\
## [PrimSpec](https://openusd.org/release/glossary.html\#id418) [](https://openusd.org/release/glossary.html\#usdglossary-primspec "Permalink to this heading")\
\
Each _composed_ [Prim](https://openusd.org/release/glossary.html#usdglossary-prim) on a [Stage](https://openusd.org/release/glossary.html#usdglossary-stage) is\
the result of potentially many _PrimSpecs_ each contributing their own scene\
description to a composite result. A PrimSpec can be thought of as an\
“uncomposed prim in a layer”. Similarly to a composed prim, a PrimSpec is a\
container for [property](https://openusd.org/release/glossary.html#usdglossary-property) data and nested\
PrimSpecs. Importantly, [composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) can\
only be applied on PrimSpecs, and those arcs that specify targets are targeting\
other PrimSpecs.\
\
## [PrimStack](https://openusd.org/release/glossary.html\#id419) [](https://openusd.org/release/glossary.html\#primstack "Permalink to this heading")\
\
A _PrimStack_ is a list of [PrimSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec) that contribute\
[opinions](https://openusd.org/release/glossary.html#usdglossary-opinions) for a composed [prim’s](https://openusd.org/release/glossary.html#usdglossary-prim) [metadata](https://openusd.org/release/glossary.html#usdglossary-metadata). This information is condensed from the\
prim’s [index](https://openusd.org/release/glossary.html#usdglossary-index), and made available through\
[UsdPrim::GetPrimStack](https://openusd.org/release/api/class_usd_prim.html#a813fc6b56b3e692d9b66ce72033ae620).\
\
## [Primvar](https://openusd.org/release/glossary.html\#id420) [](https://openusd.org/release/glossary.html\#usdglossary-primvar "Permalink to this heading")\
\
The name _primvar_ comes from RenderMan, and stands for “primitive variable”. A\
primvar is a special attribute that a renderer associates with a geometric\
primitive, and can vary (interpolate) the value of the attribute over the\
surface/volume of the primitive. In USD, you create and retrieve primvars using\
the [UsdGeomImageable](https://openusd.org/release/api/class_usd_geom_imageable.html) schema, and interact with the special primvar\
encoding using the [UsdGeomPrimvar](https://openusd.org/release/api/class_usd_geom_primvar.html) schema.\
\
There are two key aspects of Primvars:\
\
> - Primvars define a value that can vary across the primitive on which they\
> are defined, via [prescribed interpolation rules](https://openusd.org/release/api/class_usd_geom_primvar.html#Usd_InterpolationVals).\
>\
> - Taken collectively on a prim, its Primvars describe “per-primitive\
> overrides” used to communicate variables to consumers, such as providing\
> providing joint influences for UsdSkel schemas, or shader variables for\
> renderers. In the shader use-case, different renderers may communicate the\
> variables to the shaders using different mechanisms over which USD has no\
> control; Primvars simply provide the classification that any renderer\
> should use to locate potential overrides.\
\
For examples of primvars and primvar interpolation modes, see [Primvars](https://openusd.org/release/user_guides/primvars.html#primvars).\
\
## [Property](https://openusd.org/release/glossary.html\#id421) [](https://openusd.org/release/glossary.html\#usdglossary-property "Permalink to this heading")\
\
_Properties_ are the other kind of namespace object in USD (Prims being the\
first). Whereas prims provide the organization and indexing for a composed\
scene, properties contain the “real data”. There are two types of Property:\
[Attribute](https://openusd.org/release/glossary.html#usdglossary-attribute) and [Relationship](https://openusd.org/release/glossary.html#usdglossary-relationship). All properties can be ordered within their\
containing Prim (and are otherwise enumerated in dictionary order) via\
[UsdPrim::SetPropertyOrder](https://openusd.org/release/api/class_usd_prim.html#a7285692cfe69b46e890ec25756d02c37), and can host\
[Metadata](https://openusd.org/release/glossary.html#usdglossary-metadata).\
\
Sometimes it is desirable to be able to further group and organize a prim’s\
properties without introducing new child prim containers, either for locality\
reasons, or to [keep the scene description lightweight](https://openusd.org/release/maxperf.html#what-makes-a-usd-scene-heavy-expensive) to that end, properties in USD can be\
[created inside nested namespaces](https://openusd.org/release/api/class_usd_prim.html#a935381d7c7100b583fdcdb0e10dae9e6), and\
[enumerated by namespace](https://openusd.org/release/api/class_usd_prim.html#aea68cd5d8877c3faf0f8af2d0fe118f4). Here are\
some examples of namespaced properties from usd schemas:\
\
```\
#usda 1.0\
\
over MyMesh\
{\
    rel material:binding = </ModelRoot/Materials/MetalMaterial>\
    color3f[] primvars:displayColor = [ (.4, .2, .6) ]\
}\
\
```\
\
## [PropertySpec](https://openusd.org/release/glossary.html\#id422) [](https://openusd.org/release/glossary.html\#propertyspec "Permalink to this heading")\
\
Just as [PrimSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec) contain data for a prim within a\
layer, _PropertySpecs_ contain the data for a property within a\
layer. PropertySpecs are nested inside PrimSpecs; a PropertySpec can be as\
simple as a statement of a property’s existence (which, for [Attributes](https://openusd.org/release/glossary.html#usdglossary-attribute), includes its typeName), or can contain values for\
any piece of [metadata](https://openusd.org/release/glossary.html#usdglossary-metadata) authorable on properties,\
including its value. For [Relationships](https://openusd.org/release/glossary.html#usdglossary-relationship), the value\
a PropertySpec can contain is its _targets_ , which is an\
[SdfListOp<SdfPath>](https://openusd.org/release/api/class_sdf_list_op.html) For [Attributes](https://openusd.org/release/glossary.html#usdglossary-attribute), each PropertySpec can contain two independent\
values: a timeless [Default Value](https://openusd.org/release/glossary.html#usdglossary-defaultvalue), and a freely\
varying, ordered collection of [TimeSamples](https://openusd.org/release/glossary.html#usdglossary-timesample).\
\
## [PropertyStack](https://openusd.org/release/glossary.html\#id423) [](https://openusd.org/release/glossary.html\#propertystack "Permalink to this heading")\
\
A _PropertyStack_ is a list of [PropertySpecs](https://openusd.org/release/glossary.html#usdglossary-propertyspec) that\
contribute a [default](https://openusd.org/release/glossary.html#usdglossary-defaultvalue) or [timeSample](https://openusd.org/release/glossary.html#usdglossary-timesample) (for [Attributes](https://openusd.org/release/glossary.html#usdglossary-attribute)) or\
target (for [relationships](https://openusd.org/release/glossary.html#usdglossary-relationship)), _or_ any piece of\
[metadata](https://openusd.org/release/glossary.html#usdglossary-metadata), for a given [property](https://openusd.org/release/glossary.html#usdglossary-property). The information returned by\
[UsdProperty::GetPropertyStack](https://openusd.org/release/api/class_usd_property.html#a2159d3d651cd66e4fb1c724be90ed5e0) **should only be used for**\
**debugging/diagnostic purposes, not for** [value resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution), because:\
\
> 1. In the presence of [Value Clips](https://openusd.org/release/glossary.html#usdglossary-valueclips), an\
> attribute’s PropertyStack may need to be recomputed on each frame, which\
> may be expensive.\
>\
> 2. A PropertyStack does not contain the proper time-offsets that must be\
> applied to the PrimSpecs to retrieve the correct timeSample when there\
> are authored [Layer Offsets](https://openusd.org/release/glossary.html#usdglossary-layeroffset) on [references](https://openusd.org/release/glossary.html#usdglossary-references), [subLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers), or\
> [clips](https://openusd.org/release/glossary.html#usdglossary-valueclips).\
\
If your goal is to optimize repeated value resolutions on attributes, retain a\
[UsdAttributeQuery](https://openusd.org/release/api/class_usd_attribute_query.html) instead, which is designed for exactly this purpose.\
\
## [Proxy](https://openusd.org/release/glossary.html\#id424) [](https://openusd.org/release/glossary.html\#proxy "Permalink to this heading")\
\
_Proxy_ is a highly overloaded term in computer graphics… but so were all the\
alternatives we considered for the same concept in USD. “proxy” is one of the\
possible [purpose](https://openusd.org/release/glossary.html#usdglossary-purpose) values a prim can possess in the\
UsdGeom schemas. When we talk about “a proxy” for a model or part of a model,\
we mean a prim (that may have a subtree) that has purpose **proxy** , and _is_\
_paired_ with a prim whose purpose is **render.** The idea behind this pairing\
is that the **proxy** provides a set of [gprims](https://openusd.org/release/glossary.html#usdglossary-gprim) that are\
lightweight to read and draw, and provide an idea of what the full **render**\
geometry will look like, at much cheaper cost.\
\
Why not just use a “level of detail” or “standin” [VariantSet](https://openusd.org/release/glossary.html#usdglossary-variantset), rather than creating this special, different kind\
of visibility setting? The answer is twofold:\
\
> - Most clients of USD in our pipeline place a high value on bringing up a\
> complex scene for inspection and introspection as quickly as possible\
>\
> - But, they also require access to the actual data that will be used for\
> rendering, at all times.\
\
Therefore, a VariantSet is not a very attractive option for solving this display\
problem, because in order to draw the lightweight geometry, we would have\
removed the possibility of inspecting the “render quality” data, because only\
one [variant](https://openusd.org/release/glossary.html#usdglossary-variant) of a VariantSet can be composed at any\
given time, for a particular prim on a [Stage](https://openusd.org/release/glossary.html#usdglossary-stage). It is a\
fairly lightweight operation to instruct the renderer to ignore the proxies and\
image the full render geometry, when that is required.\
\
## [PseudoRoot](https://openusd.org/release/glossary.html\#id425) [](https://openusd.org/release/glossary.html\#usdglossary-pseudoroot "Permalink to this heading")\
\
A Stage’s [PseudoRoot Prim](https://openusd.org/release/api/class_usd_stage.html#a5567a555616cd3bf5fd2fc787f31accc) is a contrivance\
that allows every [UsdStage](https://openusd.org/release/glossary.html#usdglossary-stage) to contain a single\
tree of [prims](https://openusd.org/release/glossary.html#usdglossary-prim) rather than a forest. See [Namespace](https://openusd.org/release/glossary.html#usdglossary-namespace) for further details.\
\
## [Purpose](https://openusd.org/release/glossary.html\#id426) [](https://openusd.org/release/glossary.html\#usdglossary-purpose "Permalink to this heading")\
\
_Purpose_ is a builtin attribute of the [UsdGeomImageable](https://openusd.org/release/api/class_usd_geom_imageable.html)\
schema, and is a concept we have found useful in our pipeline for classifying\
geometry into categories that can each be independently included or excluded\
from traversals of prims on a stage, such as rendering or bounding-box\
computation traversals. In essence, it provides client-driven “visibility\
categories” as gates on a scenegraph traversal.\
\
The fallback purpose, **\*default\*** indicates that a prim has “no special\
purpose” and should generally be included in all traversals. Subtrees rooted at\
a prim with purpose **\*render\*** should generally only be included when\
performing a “final quality” render. Subtrees rooted at a prim with purpose\
**\*proxy\*** should generally only be included when performing a lightweight\
proxy render (such as OpenGL). Finally, subtrees rooted at a prim with purpose\
**\*guide\*** should generally only be included when an interactive application\
has been explicitly asked to “show guides”.\
\
For a discussion of the motivation for _purpose_, see [Proxy](https://openusd.org/release/glossary.html#usdglossary-proxy).\
\
## [References](https://openusd.org/release/glossary.html\#id427) [](https://openusd.org/release/glossary.html\#usdglossary-references "Permalink to this heading")\
\
After [SubLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers), _References_ are the next\
most-basic and most-important [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs).\
Because a [PrimSpec](https://openusd.org/release/glossary.html#usdglossary-primspec) can apply an entire [list](https://openusd.org/release/glossary.html#usdglossary-listediting) of References, References _can_ be used to achieve\
a similar kind of layering of data, when one knows exactly which prims need to\
be layered (and with some differences in how the [participating opinions will be\\
resolved](https://openusd.org/release/glossary.html#usdglossary-livrpsstrengthordering)).\
\
But the primary use for References is to compose smaller units of scene\
description into larger _aggregates_ , building up a [namespace](https://openusd.org/release/glossary.html#usdglossary-namespace) that includes the “encapsulated” result of composing\
the scene description targeted by a reference. Following is a simple example of\
referencing, with overrides.\
\
We start with a trivial model asset, Marble. Note that, for brevity, we\
are eliding some of the key data usually found in published assets (such as\
[AssetInfo](https://openusd.org/release/glossary.html#usdglossary-assetinfo), shading of any kind, [Inherits](https://openusd.org/release/glossary.html#usdglossary-inherits), a [Payload](https://openusd.org/release/glossary.html#usdglossary-payload), detailed model\
substructure).\
\
Marble.usd, defines a single, green marble [](https://openusd.org/release/glossary.html#id336 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    defaultPrim = "Marble"\
)\
\
def Xform "Marble" (\
    kind = "component"\
)\
{\
    def Sphere "marble_geom"\
    {\
        color3f[] primvars:displayColor = [ (0, 1, 0) ]\
    }\
}\
\
```\
\
Now we want to create a collection of marbles, by referencing the Marble\
asset multiple times, and overriding some of the referenced properties to make\
each instance unique.\
\
MarbleCollection.usd, an assembly of referenced Marble assets [](https://openusd.org/release/glossary.html#id337 "Permalink to this code")\
\
```\
#usda 1.0\
\
def Xform "MarbleCollection" (\
    kind = "assembly"\
)\
{\
    def "Marble_Green" (\
            references = @Marble.usd@\
        )\
    {\
        double3 xformOp:translate = (-10, 0, 0)\
        uniform token[] xformOpOrder = [ "xformOp:translate" ]\
    }\
\
    def "Marble_Red" (\
        references = @Marble.usd@\
    )\
    {\
        double3 xformOp:translate = (5, 0, 0)\
        uniform token[] xformOpOrder = [ "xformOp:translate" ]\
\
        over "marble_geom"\
        {\
            color3f[] primvars:displayColor = [ (1, 0, 0) ]\
        }\
    }\
}\
\
```\
\
To understand the results, we’ll examine the result of [flattening a Stage](https://openusd.org/release/glossary.html#usdglossary-flatten) opened for MarbleCollection.usd, which\
provides us with the same namespace and resolved property values that the\
originating Stage would, with all of the composition arcs “baked out”.\
\
FlattenedMarbleCollection.usd demonstrates how references combine namespaces [](https://openusd.org/release/glossary.html#id338 "Permalink to this code")\
\
```\
#usda 1.0\
\
def Xform "MarbleCollection" (\
    kind = "assembly"\
)\
{\
    def Xform "Marble_Green" (\
        kind = "component"\
    )\
    {\
        double3 xformOp:translate = (-10, 0, 0)\
        uniform token[] xformOpOrder = [ "xformOp:translate" ]\
\
        def Sphere "marble_geom"\
        {\
            color3f[] primvars:displayColor = [ (0, 1, 0) ]\
        }\
    }\
\
    def Xform "Marble_Red" (\
        kind = "component"\
    )\
    {\
        double3 xformOp:translate = (5, 0, 0)\
        uniform token[] xformOpOrder = [ "xformOp:translate" ]\
\
        def Sphere "marble_geom"\
        {\
            color3f[] primvars:displayColor = [ (1, 0, 0) ]\
        }\
    }\
}\
\
```\
\
**Things to note:**\
\
> - In the composed namespace, the prim name Marble is gone, since the\
> references allowed us to perform a prim name-change on the prim targeted\
> by the reference. This is a key feature of references, since without it,\
> we would be unable to reference the same asset more than once within any\
> given prim scoping, because sibling prims must be uniquely named to form a\
> proper namespace.\
>\
> - Even though the asset prim named /Marble/marble\_geom shows up\
> twice in the flattened scene, which indicates that there were indeed two\
> distinct prims on the Stage, when we opened a stage for the original\
> MarbleCollection.usd, the file Marble.usd was only\
> opened once and shared by both references. For deeper sharing of\
> referenced assets, in which the prims themselves are also shared, see\
> [Instancing](https://openusd.org/release/glossary.html#usdglossary-instancing) .\
>\
> - References can apply a [Layer Offset](https://openusd.org/release/glossary.html#usdglossary-layeroffset) to\
> offset and scale the time-varying data contained in the referenced\
> layer(s).\
>\
> - References can target any prim in a [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack), excepting ancestors of the prim containing\
> the reference, if the reference is an _internal reference_ targeting the\
> same LayerStack in which the reference is authored. **When targeting**\
> **sub-root prims, however, there is the potential for surprising behavior**\
> unless you are aware of and understand the ramifications. One such\
> ramification is that if the targeted sub-root prim has an ancestor prim\
> that contains a [VariantSet](https://openusd.org/release/glossary.html#usdglossary-variantset), the referencer\
> will have no ability to express a selection for that VariantSet. For a\
> more complete discussion of the ramifications of referencing sub-root\
> prims, see the [UsdReferences class documentation](https://openusd.org/release/api/class_usd_references.html).\
\
See [List Editing](https://openusd.org/release/glossary.html#usdglossary-listediting) for the rules by which\
references can be combined within a [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack)\
\
## [Relationship](https://openusd.org/release/glossary.html\#id428) [](https://openusd.org/release/glossary.html\#usdglossary-relationship "Permalink to this heading")\
\
A _Relationship_ is a “namespace pointer” that is robust in the face of\
[composition arcs](https://openusd.org/release/glossary.html#usdglossary-compositionarcs), which means that when you\
ask USD for a relationship’s targets, USD will perform all the necessary\
namespace-manipulations required to translate the **authored** target value into\
the scene-level namespace. Relationships are used throughout the USD schemas;\
perhaps most visibly in the [UsdShadeMaterialBindingAPI schema’s binding\\
of gprims to their associated Materials](https://openusd.org/release/api/class_usd_shade_material_binding_a_p_i.html).\
Relationships can have multiple targets, as, for instance, the relationships in\
a [UsdCollectionAPI](https://openusd.org/release/api/class_usd_collection_a_p_i.html) target all of the objects that belong to the\
named collection; therefore, relationships are [List Edited](https://openusd.org/release/glossary.html#usdglossary-listediting). Following is an example that demonstrates how a\
relationship’s targets must be remapped to provide useful pointers.\
\
Let’s enhance the asset example from the [References](https://openusd.org/release/glossary.html#usdglossary-references)\
entry to have a shading Material and binding:\
\
Marble.usd, with a bound Material [](https://openusd.org/release/glossary.html#id339 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    defaultPrim = "Marble"\
)\
\
def Xform "Marble" (\
    kind = "component"\
)\
{\
    def Sphere "marble_geom"\
    {\
        rel material:binding = </Marble/GlassMaterial>\
        color3f[] primvars:displayColor = [ (0, 1, 0) ]\
    }\
\
    def Material "GlassMaterial"\
    {\
        # Interface inputs, shading networks, etc.\
    }\
}\
\
```\
\
Now, because each marble in the MarbleCollection.usd scene has its\
own copy of the GlassMaterial prim, we expect that when we:\
\
Resolving referenced relationships [](https://openusd.org/release/glossary.html#id340 "Permalink to this code")\
\
```\
stage = Usd.Stage.Open("MarbleCollection.usd")\
greenMarbleGeom = stage.GetPrimAtPath("/Marble_Collection/Marble_Green/marble_geom")\
print(UsdShade.MaterialBindingAPI(greenMarbleGeom).GetDirectBindingRel().GetTargets())\
\
```\
\
we will get:\
\
> /MarbleCollection/Marble\_Green/GlassMaterial\
\
as the result, even though that was not the authored value in Marble.usd.\
\
## [Relocates](https://openusd.org/release/glossary.html\#id429) [](https://openusd.org/release/glossary.html\#relocates "Permalink to this heading")\
\
_Relocates_ is a [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) that maps\
a prim [path](https://openusd.org/release/glossary.html#usdglossary-path) defined in a remote\
[LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack) (i.e. across a composition arc) to a\
new path location in the local namespace.\
\
Relocates are defined in layer metadata, as a list of source path to target\
path mappings. Note that these paths can only be prim paths, not property paths.\
\
example relocates defined in layer metadata [](https://openusd.org/release/glossary.html#id341 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    relocates = {\
        </CharANewVersion/Clothing> : </CharACurrent/TestClothing>,\
        </EnvA/Trees> : </AlternateEnv/ParkA/Trees>\
    }\
)\
\
```\
\
Relocates let you rename or reparent prims in situations where you would not be\
able to edit the prims directly. Normally, prims with underlying PrimSpecs from\
composition arcs cannot be directly reparented. While it’s possible to reparent\
the underlying “composition source” prims directly, such a change would be a\
destructive change that would affect all other instances that share that scene\
description. Relocates provides a way to _non-destructively_ reparent or rename\
prims by specifying a mapping of source namespace paths to target namespace\
paths in the local namespace, ensuring that the source of the composition arc is\
not modified.\
\
As an example, if you had layer refLayer.usda with the following\
prims:\
\
refLayer.usda [](https://openusd.org/release/glossary.html#id342 "Permalink to this code")\
\
```\
def "PrimA" ()\
{\
    def "PrimAChild" ()\
    {\
        uniform string testString = "test"\
        float childValue = 3.5\
    }\
}\
\
```\
\
In another layer, main.usda, “PrimA” is referenced:\
\
main.usda [](https://openusd.org/release/glossary.html#id343 "Permalink to this code")\
\
```\
def "MainPrim" (\
    prepend references = @refLayer.usda@</PrimA>\
)\
{\
}\
\
```\
\
You cannot directly rename or reparent /MainPrim/PrimAChild. However,\
you can provide a relocates mapping (in main.usda) of\
MainPrim/PrimAChild to another path in the local namespace:\
\
relocates added to main.usda [](https://openusd.org/release/glossary.html#id344 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    relocates = {\
        </MainPrim/PrimAChild> : </MainPrim/RenamedPrimAChild>\
    }\
)\
\
```\
\
This renames /MainPrim/PrimAChild to\
/MainPrim/RenamedPrimAChild without affecting the\
refLayer.usda layer.\
\
You could then add an override for /MainPrim/RenamedPrimAChild in\
main.usda. Note that the override uses the _relocated path_:\
\
override added to main.usda [](https://openusd.org/release/glossary.html#id345 "Permalink to this code")\
\
```\
def "MainPrim" (\
    prepend references = @refLayer.usda@</PrimA>\
)\
{\
    over RenamedPrimAChild\
    {\
        float childValue = 5.2\
    }\
}\
\
```\
\
The resulting stage composition will apply the override to the relocated prim\
reference:\
\
flattened main.usda [](https://openusd.org/release/glossary.html#id346 "Permalink to this code")\
\
```\
def "MainPrim"\
{\
    def "RenamedPrimAChild"\
    {\
        float childValue = 5.2\
        uniform string testString = "test"\
    }\
}\
\
```\
\
**Things to note:**\
\
> - You cannot relocate a root prim. In other words, the source path for a\
> relocates cannot be a root prim. This is because it’s impossible for a\
> root prim to be introduced by an ancestral composition arc, and relocates\
> can only used to map prims introduced via composition arcs.\
>\
> - When a source path is relocated, that original source path is considered\
> _no longer valid in the current namespace_. Any local opinions authored\
> on a source path will generate a “invalid option at relocation source\
> path” error. See\
> [local opinions not allowed at source paths](https://openusd.org/release/glossary.html#usdglossary-relocates-source-invalid)\
> below for more details.\
>\
> - Source and target paths must be complete scene paths. Paths with variant\
> selections (e.g. /Prim{var=sel}Child) are not supported.\
>\
> - Relocates that would create invalid or conflicting namespace paths are not\
> allowed, such as:\
>\
>\
> > - Relocating a prim to an existing ancestor:\
> > Prim/Child/Grandchild : Prim/Child is not\
> > allowed.\
> >\
> > - Relocating a prim to a descendant of the source path:\
> > /Prim/Child : /Prim/Child/Grandchild is not\
> > allowed.\
> >\
> > - Relocating the same source path to multiple targets, or relocating\
> > multiple source paths to the same target.\
> >\
> > - Relocating a prim to the source path of a different relocate in the\
> > same namespace:\
> > /Prim/Prim1 : /Prim/Prim2,\
> > /Prim/Prim2 : /Prim/Prim3 is not allowed.\
> > “Transitive” relocates must be collapsed into the smallest relocation,\
> > e.g. for the previous example,\
> > /Prim/Prim1 : /Prim/Prim3 should be used instead.\
> > This applies to relocates in the same LayerStack.\
>\
> - If a relocate has “ancestral relocates” (e.g. an ancestor prim that has\
> also been relocated), the relocate source path must use the ancestral\
> relocated path. For example, if you have /Root referencing\
> /Ref, and /Ref also references /Ref2,\
> if /Root/Ref is relocated to /Root/RefRelocated,\
> any additional relocate that would use /Root/Ref/Ref2 as a\
> source path must use the relocated path\
> /Root/RefRelocated/Ref2.\
\
With respect to\
[composition strength ordering](https://openusd.org/release/glossary.html#usdglossary-livrpsstrengthordering),\
relocates is stronger than [References](https://openusd.org/release/glossary.html#usdglossary-references), but weaker than\
[VariantSet](https://openusd.org/release/glossary.html#usdglossary-variantset). In the previous example, if we had an authored\
opinion at a relocates target location that used the [Inherits](https://openusd.org/release/glossary.html#usdglossary-inherits)\
composition arc (which is stronger than relocates) we might have a\
main.usda layer that looks like the following:\
\
main.usda with added class and inherits [](https://openusd.org/release/glossary.html#id347 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    relocates = {\
        </MainPrim/PrimAChild> : </MainPrim/RenamedPrimAChild>\
    }\
)\
\
class "WorkClass"\
{\
    float childValue = 20.5\
    uniform string testString = "from WorkClass"\
}\
\
def "MainPrim" (\
    prepend references = @refLayer.usda@</PrimA>\
)\
{\
    def "RenamedPrimAChild"\
    (\
        inherits = </WorkClass>\
    )\
    {\
    }\
}\
\
```\
\
When the stage is composed, the inherited opinions for\
MainPrim/RenamedPrimAChild will be stronger:\
\
flattened main.usda with inherits and relocates applied [](https://openusd.org/release/glossary.html#id348 "Permalink to this code")\
\
```\
def "MainPrim"\
{\
    def "RenamedPrimAChild"\
    {\
        float childValue = 20.5\
        uniform string testString = "from WorkClass"\
    }\
}\
\
```\
\
**Relocates and inherits**\
\
A relocated prim will still inherit the same opinions it would have had it not\
been relocated. This can result in some subtle composition behavior.\
\
For example, we have a layer model.usda that defines\
/ClassA and has prim /Model that inherits from\
/ClassA. It also has a relocate for /Model/Rig/LRig to\
/Model/Anim/LAnim:\
\
model.usda with ClassA and Model that inherits from ClassA [](https://openusd.org/release/glossary.html#id349 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    relocates = {\
        </Model/Rig/LRig>: </Model/Anim/LAnim>\
    }\
)\
\
class "ClassA"\
{\
    def "Rig"\
    {\
        def "LRig"\
        {\
            uniform token modelClassALRig = "test"\
        }\
    }\
\
    def "Anim"\
    {\
        def "LAnim"\
        {\
            uniform token modelClassALAnim = "test"\
        }\
    }\
}\
\
def "Model" (\
    inherits = </ClassA>\
)\
{\
}\
\
```\
\
We reference /Model in another layer, root.usda, which\
also has a /ClassA class:\
\
root.usda [](https://openusd.org/release/glossary.html#id350 "Permalink to this code")\
\
```\
def "Model_1" (\
    references = @./model.usda@</Model>\
)\
{\
}\
\
class "ClassA"\
{\
    def "Rig"\
    {\
        def "LRig"\
        {\
            uniform token rootClassALRig = "test"\
        }\
    }\
\
    def "Anim"\
    {\
        def "LAnim"\
        {\
            uniform token rootClassALAnim = "test"\
        }\
    }\
}\
\
```\
\
If we load root.usda and inspect the flattened stage,\
/Model/Rig/LRig in model.usda has inherited from\
/ClassA/Rig/LRig even though it was relocated to\
/Model/Anim/LAnim in that layer, and does _not_ inherit opinions from\
/ClassA/Anim/LAnim. However, note that /Model\_1/Anim/LAnim\
in the root.usda layer does inherit from the layer’s\
/ClassA/Anim/LAnim.\
\
flattened root.usda [](https://openusd.org/release/glossary.html#id351 "Permalink to this code")\
\
```\
def "Model_1"\
{\
    def "Rig"\
    {\
    }\
\
    def "Anim"\
    {\
        def "LAnim"\
        {\
            uniform token modelClassALRig = "test"\
            uniform token rootClassALAnim = "test"\
            uniform token rootClassALRig = "test"\
        }\
    }\
}\
\
```\
\
**Relocates and ancestral arcs during composition**\
\
One aspect of relocates and composition is that relocates will _ignore_\
all ancestral arcs _except variant arcs_ when we build the\
[PrimIndex](https://openusd.org/release/glossary.html#usdglossary-index) for a prim. So, if you had a layer that\
relocates a prim to be the child of a prim with an ancestral inherits arc:\
\
layer with ancestral inherits and relocates [](https://openusd.org/release/glossary.html#id352 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    relocates = {\
        </PrimA/Child>: </PrimWithInherits/Child>\
    }\
)\
\
def "ClassA"\
(\
)\
{\
    def "Child"\
    {\
        uniform token testString = "from ClassA/Child"\
        uniform token classAChildString = "test"\
    }\
}\
\
def "RefPrim"\
(\
)\
{\
    def "Child"\
    {\
        uniform token testString = "from RefPrim/Child"\
        uniform token refPrimChildString = "test"\
    }\
}\
\
def "PrimA"\
(\
    prepend references = </RefPrim>\
)\
{\
}\
\
def "PrimWithInherits"\
(\
    inherits = </ClassA>\
)\
{\
}\
\
```\
\
With the relocates for /PrimA/Child to\
/PrimWithInherits/Child, the ancestral opinions from\
/ClassA/Child are ignored.\
\
flattened PrimWithInherits [](https://openusd.org/release/glossary.html#id353 "Permalink to this code")\
\
```\
def "PrimWithInherits"\
{\
    def "Child"\
    {\
        uniform token refPrimChildString = "test"\
        uniform token testString = "from RefPrim/Child"\
    }\
}\
\
```\
\
However, as mentioned earlier, ancestral _variant_ arcs will still compose with\
relocates. If we introduce ancestral opinions from a variant in\
/PrimWithInherits instead of using inherits:\
\
replace inherits with variantset in PrimWithInherits [](https://openusd.org/release/glossary.html#id354 "Permalink to this code")\
\
```\
def "PrimWithInherits"\
(\
    # Removed inherits of ClassA\
    # Added variantSet and selection with authored Child opinions\
    variants = {\
        string varSet = "Set1"\
    }\
    prepend variantSets = "varSet"\
)\
{\
    variantSet "varSet" = {\
        "Set1" ()\
        {\
            def "Child"\
            {\
                uniform token testString = "from varSet Child"\
                uniform token varChildString = "test"\
            }\
        }\
        "Set2" ()\
        {\
        }\
    }\
}\
\
```\
\
The ancestral opinions from the selected variant will be applied:\
\
flattened PrimWithInherits with ancestral variant opinions [](https://openusd.org/release/glossary.html#id355 "Permalink to this code")\
\
```\
def "PrimWithInherits"\
{\
    def "Child"\
    {\
        uniform token refPrimChildString = "test"\
        uniform token testString = "from varSet Child"\
        uniform token varChildString = "test"\
    }\
}\
\
```\
\
See [composition strength ordering](https://openusd.org/release/glossary.html#usdglossary-livrpsstrengthordering)\
for more details on relocates and composition.\
\
**Local opinions not allowed at source paths**\
\
When a source path is relocated, that original source path is considered\
no longer valid in the current namespace. So, if you relocated\
/PrimA/Child to /PrimA/NewChild, you cannot have any\
local opinions authored at /PrimA/Child. This avoids ambiguity and\
ensures there’s exactly one location (the target path) in the namespace to\
express opinions about a given object.\
\
## [Root LayerStack](https://openusd.org/release/glossary.html\#id430) [](https://openusd.org/release/glossary.html\#usdglossary-rootlayerstack "Permalink to this heading")\
\
Every [Stage](https://openusd.org/release/glossary.html#usdglossary-stage) has a “root” [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack), comprised of the LayerStack defined by the root\
[Layer](https://openusd.org/release/glossary.html#usdglossary-layer) on which the Stage was opened, **appended** to the\
LayerStack defined by the stage’s [Session Layer](https://openusd.org/release/glossary.html#usdglossary-sessionlayer)\
(the method for enumerating the layers in a Stage’s root LayerStack,\
[UsdStage::GetLayerStack](https://openusd.org/release/api/class_usd_stage.html#a9dfcae900fd07e3344e087ea1bd478cb), provides the option of eliding the Session\
Layer’s contributions). The root LayerStack is special/important for two\
reasons:\
\
> 1. The [prims](https://openusd.org/release/glossary.html#usdglossary-prim) declared in root layers are the only\
> ones locatable using the same [paths](https://openusd.org/release/glossary.html#usdglossary-path) that identify\
> composed prims on the Stage. Currently, an [Edit Target](https://openusd.org/release/glossary.html#usdglossary-edittarget) can only target [PrimSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec) in the root LayerStack, although we hope to\
> relax that restriction eventually.\
>\
> 2. It is the layers of the root LayerStack that are the most useful in\
> facilitating shared workflows using USD. Cooperating departments and/or\
> artists can each manage their own layer(s) in the root LayerStack of an\
> asset, sequence, or shot, and their work will combine in intuitive ways\
> with that of other artists working in the same context in their own\
> layers.\
\
## [Schema](https://openusd.org/release/glossary.html\#id431) [](https://openusd.org/release/glossary.html\#usdglossary-schema "Permalink to this heading")\
\
USD defines a _schema_ as an object whose purpose is to author and retrieve\
structured data from some [UsdObject](https://openusd.org/release/api/class_usd_object.html). Most schemas found in the core\
are “prim schemas”, which are further refined into [IsA Schemas](https://openusd.org/release/glossary.html#usdglossary-isaschema) and [API Schemas](https://openusd.org/release/glossary.html#usdglossary-apischema), for\
which the USD distribution provides [tools for code generation](https://openusd.org/release/api/_usd__page__generating_schemas.html) to create your own schemas. However,\
there are several examples of “property schemas”, also, such as\
[UsdGeomPrimvar](https://openusd.org/release/api/class_usd_geom_primvar.html) and [UsdShadeInput](https://openusd.org/release/api/class_usd_shade_input.html). Schemas are\
lightweight objects we create to wrap a [UsdObject](https://openusd.org/release/api/class_usd_object.html), as and when needed,\
to robustly interrogate and author scene description. We also use schema classes\
to package type/schema-based **computations,** such as\
[UsdGeomImageable::ComputeVisibility](https://openusd.org/release/api/class_usd_geom_imageable.html#ac0bfa1554d9b57303c14f71d084c578d).\
\
## [Session Layer](https://openusd.org/release/glossary.html\#id432) [](https://openusd.org/release/glossary.html\#usdglossary-sessionlayer "Permalink to this heading")\
\
Each [UsdStage](https://openusd.org/release/glossary.html#usdglossary-stage) can be created with a _session layer_ that\
provides for “scratch space” to configure, override, and experiment with the\
data contained in files backing the stage. If requested or provided at stage\
creation-time, a session layer participates fully in the stage’s [composition](https://openusd.org/release/glossary.html#usdglossary-composition), as the strongest layer in the stage’s [Root\\
LayerStack](https://openusd.org/release/glossary.html#usdglossary-rootlayerstack), and can possess its own [SubLayers](https://openusd.org/release/glossary.html#usdglossary-sublayers). Session layers generally embody “application state”,\
and, if saved, would be saved as part of application state rather than as part\
of the data set they modify. [usdview](https://openusd.org/release/toolset.html#usdview) creates a sesssion layer,\
into which are targeted all [VariantSet](https://openusd.org/release/glossary.html#usdglossary-variantset) selections,\
[vis/invis](https://openusd.org/release/glossary.html#usdglossary-visibility) opinions, and [activation/deactivation](https://openusd.org/release/glossary.html#usdglossary-active-inactive) operations provided by the GUI. In keeping with\
the view of session-layer as application state rather than asset data,\
[UsdStage::Save](https://openusd.org/release/api/class_usd_stage.html#adefa2f7ebfc4d8c09f0cd54419aa36c4) **does not save its stage’s session layer(s)**.\
\
To edit content in a session layer, get the layer’s edit target using\
[stage->GetEditTargetForLocalLayer(stage->GetSessionLayer())](https://openusd.org/release/api/class_usd_stage.html#a39a51571cd00e6f38572f7f55a101cf5) and set that target in the stage by\
calling [UsdStage::SetEditTarget](https://openusd.org/release/api/class_usd_stage.html#ac90008efb0f2b624fcff43de00188792) or creating a\
[UsdEditContext](https://openusd.org/release/api/class_usd_edit_context.html).\
\
## [Specializes](https://openusd.org/release/glossary.html\#id433) [](https://openusd.org/release/glossary.html\#usdglossary-specializes "Permalink to this heading")\
\
_Specializes_ is a [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) that allows\
a _specialized_ prim to be continuously refined from a _base_ prim, through\
unlimited levels of referencing. A specialized prim will contain all of the\
scene description contained in the base prim it specializes (including the\
entire namespace hierarchy rooted at the base prim), but any opinion expressed\
directly on the specialized prim will always be stronger than any opinion\
expressed on the base prim, **in any referencing context**. In behavior,\
specializes is very similar to [inherits](https://openusd.org/release/glossary.html#usdglossary-inherits), with the\
key difference being the relative strength of referenced opinions vs\
“inherited/specialized” opinions, which leads to very different uses for the\
two.\
\
Let us examine an example inspired by the first uses of the specializes arc at\
Pixar: specializing materials in our shading schema. We wish to publish an\
asset that contains several types of metallic materials, all of which should\
maintain their basic “metalness”, even as the definition of metal may change in\
referencing contexts. For brevity, we focus on one particular specialization, a\
corroded metal; we also leave out many of the schema details of how materials\
and their shaders interact.\
\
Robot.usd [](https://openusd.org/release/glossary.html#id356 "Permalink to this code")\
\
```\
#usda 1.0\
\
def Xform "Robot"\
{\
    def Scope "Materials"\
    {\
        def Material "Metal"\
        {\
            # Interface inputs drive shader parameters of the encapsulated\
            # network. We are not showing the connections, nor how we encode\
            # that the child Shader "Surface" is the primary output for the\
            # material.\
            float inputs:diffuseGain = 0\
            float inputs:specularRoughness = 0\
\
            def Shader "Surface"\
            {\
                asset info:id = @PxrSurface@\
            }\
        }\
\
        def Material "CorrodedMetal" (\
            specializes = </Robot/Materials/Metal>\
        )\
        {\
            # specialize roughness...\
            float inputs:specularRoughness = 0.2\
\
            # Adding a pattern to drive Surface bump\
            def Shader "Corrosion"\
            {\
                asset info:id = @PxrOSL@\
                vector3f outputs:disp\
            }\
\
            over "Surface"\
            {\
                # Override that would connect specularBump to Corrosion\
                # pattern's "outputs:disp" attribute\
            }\
        }\
    }\
}\
\
```\
\
**Notes:**\
\
> - The above example is not realistic at all regarding how you would actually\
> design a Metal or CorrodedMetal material!\
>\
> - Specializes can target any prim **on the stage** that is neither an\
> ancestor nor descendant of the specializing prim.\
>\
> - In the example above, replacing the _specializes_ with [inherits](https://openusd.org/release/glossary.html#usdglossary-inherits) will produce the same composed result - try it!\
\
In the sameRobot.usd asset, we might define many more materials,\
some of which are “siblings” of CorrodedMetal in that they specialize\
Metal, and some that further specialize CorrodedMetal, since\
specialization is not limited to any depth. The unique behavior of specializes\
only becomes evident, however, in a referencing context, so let us try one.\
\
RobotScene.usd [](https://openusd.org/release/glossary.html#id357 "Permalink to this code")\
\
```\
#usda 1.0\
\
def Xform "World"\
{\
    def Xform "Characters"\
    {\
        def "Rosie" (\
            references = @./Robot.usd@</Robot>\
        )\
        {\
            over "Materials"\
            {\
                over "Metal"\
                {\
                     float inputs:diffuseGain = 0.3\
                     float inputs:specularRoughness = 0.1\
                }\
            }\
        }\
    }\
}\
\
```\
\
If you examine the [flattened](https://openusd.org/release/glossary.html#usdglossary-flatten) RobotScene.usd you will see the effect of specializes on the\
specialized /World/Characters/Rosie/Materials/CorrodedMetal prim: we\
overrode both diffuseGain and specularRoughness on the base\
Metal material, but only the diffuseGain propagates onto\
/World/Characters/Rosie/Materials/CorrodedMetal, because\
specularRoughness was already refined on the **referenced**/Robot/Materials/CorrodedMetal prim. This also demonstrates the\
difference between specializes and [inherits](https://openusd.org/release/glossary.html#usdglossary-inherits): if you\
change the specializes arc to inherits in Robot.usd and\
recompose the scene, you will see that **both** diffuseGain and\
specularRoughness propagate onto\
/World/Characters/Rosie/Materials/CorrodedMetal.\
\
The _specializes_ behavior is desirable in this context of building up many\
unique refinements of something whose base properties we may want to continue to\
update as assets travel down the pipeline, but **without** changing anything\
that makes the refinements unique. What if we **do** want to broadcast an edit\
on the Metal material to all of the Materials that specialize it? All we need\
do is use _inherits_ and _specializes_ together: make each Metal itself and each\
specialized material (such as CorrodedMaterial) inherit from a /\_class\_Metal\
prim. Now any overrides added in /\_class\_Metal will propagate to each\
specialized material regardless of whether they have expressed any opinion about\
it in referenced layers, themselves.\
\
## [Specifier](https://openusd.org/release/glossary.html\#id434) [](https://openusd.org/release/glossary.html\#usdglossary-specifier "Permalink to this heading")\
\
Every [PrimSpec](https://openusd.org/release/glossary.html#usdglossary-primspec) possesses a _specifier_, which\
conveys the author’s (authoring tool’s) intent for how the PrimSpec should be\
consumed and interpreted in a composed scene. There are three possible values\
for a prim’s specifier in USD, defined by [SdfSpecifier](https://openusd.org/release/api/usd_2sdf_2types_8h.html#a18e79a065a46596da11ad51b2c85cad7):\
\
> - [def](https://openusd.org/release/glossary.html#usdglossary-def) \- a concrete, defined prim\
>\
> - [over](https://openusd.org/release/glossary.html#usdglossary-over) \- a speculative override\
>\
> - [class](https://openusd.org/release/glossary.html#usdglossary-class) \- prims from which other prims [inherit](https://openusd.org/release/glossary.html#usdglossary-inherits)\
\
A prim’s **resolved specifier** on a UsdStage determines which kinds of\
traversals (as defined by [“prim flags”](https://openusd.org/release/api/prim_flags_8h.html#Usd_PrimFlags)) will visit\
the prim. The most common, default traversals, which are meant to be used for\
rendering and other common scenegraph processing, will visit only **defined**,\
**non-abstract** prims.\
\
## [Stage](https://openusd.org/release/glossary.html\#id435) [](https://openusd.org/release/glossary.html\#usdglossary-stage "Permalink to this heading")\
\
A _stage_ is the USD abstraction for a scenegraph derived from a root USD file,\
and all of the referenced/layered files it [composes](https://openusd.org/release/glossary.html#usdglossary-compositionarcs). A stage always presents the composed view of\
the scene description that backs it. The [UsdStage](https://openusd.org/release/api/class_usd_stage.html) class embodies a\
Stage, and caches in memory just enough information about the composed namespace\
and backing files so that we can rapidly traverse the stage and perform\
efficient [data queries](https://openusd.org/release/glossary.html#usdglossary-valueresolution), and allow prim,\
property, and metadata editing targeted at any layer that contributes to the\
Stage’s composition. Mutated stage layers can be collectively saved to\
backing-store using [UsdStage::Save](https://openusd.org/release/api/class_usd_stage.html#adefa2f7ebfc4d8c09f0cd54419aa36c4).\
\
## [Stage Traversal](https://openusd.org/release/glossary.html\#id436) [](https://openusd.org/release/glossary.html\#stage-traversal "Permalink to this heading")\
\
Most consumption of USD data follows the pattern of traversing an open stage,\
processing prims one at a time, or several in parallel. A UsdStage allows\
for both direct and subtree range-based traversals, each with “filtering”\
according to a number of criteria.\
\
> - **direct**\
>\
> Given a [UsdPrim](https://openusd.org/release/glossary.html#usdglossary-prim) _p_, one can fetch its\
> direct children on the Stage using p.GetChild() <UsdPrim::GetChild>,\
> p.GetChildren() <UsdPrim::GetChildren>,\
> p.GetAllChildren() <UsdPrim::GetAllChildren>, and\
> p.GetFilteredChildren() <UsdPrim::GetFilteredChildren>.\
>\
> - **subtree range iteration**\
>\
> One can also fetch a [UsdPrimSubtreeRange](https://openusd.org/release/api/class_usd_prim_subtree_range.html) directly\
> from a prim via p.GetDescendants() <UsdPrim::GetDescendants>,\
> p.GetAllDescendants() <UsdPrim::GetAllDescendants>, and\
> p.GetFilteredDescendants() <UsdPrim::GetFilteredDescendants>.\
>\
>\
>\
> Note\
>\
>\
>\
> None of the Get\*Descendants() methods are wrapped to Python,\
> pending a refactoring project).\
>\
>\
>\
> Additionally, and more commonly used in USD core code, one can traverse\
> the subtree rooted at a prim by creating a [UsdPrimRange](https://openusd.org/release/api/class_usd_prim_range.html), which\
> allows for depth-first iteration with the ability to prune subtrees at any\
> point during the iteration. The [UsdStage::Traverse](https://openusd.org/release/api/class_usd_stage.html#adba675b55f41cc1b305bed414fc4f178) method\
> creates and returns a UsdPrimRange that will traverse the entire stage\
> using the “default predicate.”\
\
**Traversal Filtering via PrimFlag Predicates**\
\
All of the methods mentioned above that contain “Filtered” in their name, plus\
UsdPrimRange, allow one to specify a predicate consisting of a boolean\
combination of “flags” that test against certain (all cached, for fast access)\
states of prims. Some examples are whether a prim is defined, or active, or\
loaded, etc. For a full list of the possible prim flags and examples of how they\
can be logically combined, see [prim predicate flags](https://openusd.org/release/api/prim_flags_8h.html#Usd_PrimFlags). The remaining methods\
(e.g. GetChildren()) all use a predefined [Default Predicate](https://openusd.org/release/api/prim_flags_8h.html#a29cc22af3e47af74a71814d9cca1c1fe) that serves a common traversal pattern.\
\
## [Subcomponent](https://openusd.org/release/glossary.html\#id437) [](https://openusd.org/release/glossary.html\#usdglossary-subcomponent "Permalink to this heading")\
\
_Subcomponent_ is another [kind](https://openusd.org/release/glossary.html#usdglossary-kind) value like [component](https://openusd.org/release/glossary.html#usdglossary-component) that can be applied to a prim, but is not a “ [model\\
kind](https://openusd.org/release/glossary.html#usdglossary-model)”, and serves a different purpose. The model-kinds\
collaborate to form a [model hierarchy](https://openusd.org/release/glossary.html#usdglossary-modelhierarchy), whose\
purpose is to provide a lightweight prefix/index of a complicated scene. By\
contrast, subcomponents have no strict contiguity rules associated with them;\
rather, a subcomponent just identifies some prim inside a component that may be\
“important”. An exporter may choose to identify articulation points in a\
complicated model by labeling such prims as subcomponents (for example, the\
DoorAssemblyXform inside an architectural model). Thus\
subcomponents provide a way of setting up levels of organizational complexity\
within component models, providing intermediate interfaces/views of a model’s\
organization that sit in-between “just show me a single prim for the entire\
model” and “expand all hierarchy down to leaf prims”.\
\
As another example of how subcomponents may guide interaction behavior, the\
pxrUsdReferenceAssembly plugin for Maya uses the presence of subcomponent prims\
to guide the behavior of its “expanded” representation.\
\
## [SubLayers](https://openusd.org/release/glossary.html\#id438) [](https://openusd.org/release/glossary.html\#usdglossary-sublayers "Permalink to this heading")\
\
_SubLayers_ is the [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) used to\
construct [LayerStacks](https://openusd.org/release/glossary.html#usdglossary-layerstack). As an example, here is\
one possible combination of USD layers that could have been the source for the\
example in the [LayerStack](https://openusd.org/release/glossary.html#usdglossary-layerstack) entry, and also\
demonstrate how SubLayers supports nested LayerStacks:\
\
shot.usd [](https://openusd.org/release/glossary.html#id358 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    subLayers = [\
        @shotFX.usd@,\
        @shotAnimationBake.usd@,\
        @sequence.usd@\
    ]\
)\
\
```\
\
sequence.usd [](https://openusd.org/release/glossary.html#id359 "Permalink to this code")\
\
```\
#usda 1.0\
(\
    subLayers = [\
        @sequenceFX.usd@,\
        @sequenceLayout.usd@,\
        @sequenceDressing.usd@\
    ]\
)\
\
```\
\
Note that SubLayers can specify [Layer Offsets](https://openusd.org/release/glossary.html#usdglossary-layeroffset) to\
offset and scale time-varying data contained in the sub-layer(s)\
\
## [TimeCode](https://openusd.org/release/glossary.html\#id439) [](https://openusd.org/release/glossary.html\#usdglossary-timecode "Permalink to this heading")\
\
_TimeCodes_ are the unit-less time coordinate in USD. A [UsdTimeCode](https://openusd.org/release/api/class_usd_time_code.html)\
can encode the coordinate for a [TimeSample](https://openusd.org/release/glossary.html#usdglossary-timesample) in\
double-precision floating point, but can also encode the coordinate that maps to\
an attribute’s [Default Value](https://openusd.org/release/glossary.html#usdglossary-defaultvalue). For any given\
composed scene, defined by its root layer, the TimeCode coordinates of the\
TimeSamples contained in the scene are\
scaled to seconds by the root layer’s\
timeCodesPerSecond metadata, which can be retrieved with\
[UsdStage::GetTimeCodesPerSecond](https://openusd.org/release/api/class_usd_stage.html#a85092d7455ae894d50224e761dc6e840). This allows clients great\
flexibility to encode their TimeSamples within the range and scale that makes\
the most sense for their application, while retaining a robust mapping to “real\
time” for decoding and playback.\
\
TimeCodes can also appear in USD scenes as the timeCode metadata or\
attribute value type, and when they do, queried attribute _values_ will receive\
the same time-remapping that TimeSample coordinates do. Such timeCode-valued\
attributes can serve as “timing curves” that maintain their accuracy through\
composed layer offsets.\
\
## [TimeCodes Scaled to Real Time](https://openusd.org/release/glossary.html\#id440) [](https://openusd.org/release/glossary.html\#timecodes-scaled-to-real-time "Permalink to this heading")\
\
For a composed scene, [TimeCode](https://openusd.org/release/glossary.html#usdglossary-timecode) coordinate values\
from [TimeSamples](https://openusd.org/release/glossary.html#usdglossary-timesample) are scaled to real-time seconds\
by the root layer’s (or session layer’s) timeCodesPerSecond metadata.\
In the following example layer, the translation TimeSample on Sphere at TimeCode\
240 corresponds to 10 seconds of real time, based on the layer’s\
timeCodesPerSecond of 24.\
\
```\
#usda 1.0\
(\
    timeCodesPerSecond = 24\
    endTimeCode = 240\
    startTimeCode = 1\
)\
\
def Xform "Asset"\
{\
    def Sphere "Sphere"\
    {\
        double3 xformOp:translate.timeSamples = {\
            1: (0, 5.0, 0),\
            240: (0, -5.0, 0),\
        }\
        uniform token[] xformOpOrder = ["xformOp:translate"]\
    }\
}\
\
```\
\
If a layer specifies timeCodesPerSecond and is sublayered or referenced\
into another layer, the TimeCode values and TimeSample coordinates in the\
sublayered/referenced layer are automatically scaled to map into the timing\
defined by the root layer’s timeCodesPerSecond. If the previous example\
layer was referenced into another layer that specified a\
timeCodesPerSecond value of 48, the TimeSamples on Sphere would be\
scaled accordingly. For example, the TimeSample at TimeCode 240 would be scaled\
to TimeCode 480 to ensure that the translation still occurs at 10 seconds of\
real time.\
\
USD also provides the framesPerSecond layer metadata, however this is\
not used to directly scale TimeCodes, but instead used as an indication of the\
desired play-back rate when the animation is viewed in a playback device\
(DCC tool, usdview, etc). If the previous example layer specified a\
framesPerSecond of 12, this would _not_ change the scaling of the\
TimeSample at TimeCode 240, and instead change the playback rate in a playback\
device to march forward by two TimeCodes for each consecutive rendered frame,\
which will be held for 1/12 of a second.\
\
```\
#usda 1.0\
(\
    timeCodesPerSecond = 24\
    framesPerSecond = 12\
    endTimeCode = 240\
    startTimeCode = 1\
)\
\
```\
\
Note that framesPerSecond can be used indirectly to scale TimeCodes,\
because it is used as a fallback value for timeCodesPerSecond if\
timeCodesPerSecond is not set. The order of precedence USD uses for\
determining the timeCodesPerSecond to use is:\
\
- timeCodesPerSecond from session layer\
\
- timeCodesPerSecond from root layer\
\
- framesPerSecond from session layer\
\
- framesPerSecond from root layer\
\
- fallback value of 24\
\
\
The general best practice is to use timeCodesPerSecond to specify how\
TimeCodes are scaled to real time, and framesPerSecond if you need to\
encode a specific playback rate on playback devices, regardless of how many\
samples per second are recorded in the USD scene.\
\
Note\
\
We provide the information about framesPerSecond as fallback for\
timeCodesPerSecond primarily as a debugging aid, should you observe\
unexpected time-scaling. The fallback behavior derives only from USD’s\
relationship to Pixar’s Presto animation system.\
\
## [TimeSample](https://openusd.org/release/glossary.html\#id441) [](https://openusd.org/release/glossary.html\#usdglossary-timesample "Permalink to this heading")\
\
The term _timeSample_ is used in two related contexts in USD:\
\
> - **TimeSamples as source for** [Value Resolution](https://openusd.org/release/glossary.html#usdglossary-valueresolution)\
>\
> Each [PropertySpec](https://openusd.org/release/glossary.html#usdglossary-propertyspec) for an [Attribute](https://openusd.org/release/glossary.html#usdglossary-attribute) can contain a collection called _timeSamples_\
> that maps [TimeCode](https://openusd.org/release/glossary.html#usdglossary-timecode) coordinates to values of the\
> Attribute’s type.\
>\
> - **The time-coordinate for an Attribute**\
>\
> USD API sometimes refers to just the coordinate of a time-varying value as\
> a TimeSample; for example, [UsdAttribute::GetTimeSamples](https://openusd.org/release/api/class_usd_attribute.html#a0aba275933a77f28ab44b750964aa9a2) and\
> [UsdAttribute::GetTimeSamplesInInterval](https://openusd.org/release/api/class_usd_attribute.html#a47ff5d05a3872ef5061e8908fde2d559) return\
> a simple vector of time coordinates at which samples may be resolved on\
> the attribute.\
\
## [Typed Schema](https://openusd.org/release/glossary.html\#id442) [](https://openusd.org/release/glossary.html\#typed-schema "Permalink to this heading")\
\
Synonym for [IsA Schema](https://openusd.org/release/glossary.html#usdglossary-isaschema).\
\
## [User Properties](https://openusd.org/release/glossary.html\#id443) [](https://openusd.org/release/glossary.html\#user-properties "Permalink to this heading")\
\
userProperties: is a [property namespace](https://openusd.org/release/glossary.html#usdglossary-property)\
recognized by the majority of DCC plugins as a place to look for “extra custom\
properties” that are too informal to warrant creating an [API Schema](https://openusd.org/release/glossary.html#usdglossary-apischema), but which you might need access to in the DCC’s,\
and want to ensure they get round-tripped back to USD. We plan to create a\
UsdUserPropertiesAPI schema to aid in authoring and enumerating user\
properties, but creating a property in the userProperties: namespace\
using UsdPrim::CreateAttribute() (not all importers may handle custom\
relationships properly) is sufficient.\
\
## [Value Clips](https://openusd.org/release/glossary.html\#id444) [](https://openusd.org/release/glossary.html\#usdglossary-valueclips "Permalink to this heading")\
\
_Value Clips_ are a feature that allows one to partition varying attribute\
[timeSample](https://openusd.org/release/glossary.html#usdglossary-timesample) overrides into multiple files, and\
combine them in a manner similar to how non-linear video editing tools allow\
one to combine video clips. Clips are especially useful for solving two\
important problems in computer graphics production pipelines:\
\
> 1. **Crowd/background animation at scale**\
>\
> Crowd animators will often create animation clips that can apply to many\
> background characters, and be sequenced and cycled to generate a large\
> variety of animation. USD clips provide the ability to encode the\
> sequencing and non-uniform time-mapping of baked animation clips that\
> this task requires.\
>\
> 2. **File-per-frame “big data”**\
>\
> The results of some simulations and other types of sequentially-generated\
> special effects generate so much data that it is most practical for the\
> simulator to write out each time-step or frame’s worth of data into a\
> different file. USD Clips make it possible to stitch all of these files\
> together into a continuous (even though the data may itself be\
> topologically varying over time) animation, without needing to move,\
> merge, or perturb the files that the simulator produced. The USD toolset\
> includes a utility [usdstitchclips](https://openusd.org/release/toolset.html#usdstitchclips) that efficiently\
> assembles a sequence of file-per-frame layers into a Value Clips\
> representation.\
\
The key advantage of the clips feature is that the resulting resolved animation\
on a UsdStage is indistinguishable from data collected or aggregated into\
a single layer. In other words, consuming clients can be completely unaware of\
the existence of clips: there is no special schema or API required to access the\
data. The disadvantages of using clips are:\
\
> 1. Encoding clips on a stage is more complicated than simply recording\
> samples on attributes, or adding references (see [UsdClipsAPI](https://openusd.org/release/api/class_usd_clips_a_p_i.html)\
> for details on encoding)\
>\
> 2. There is some performance overhead associated with the use of clips, both\
> in the number of files that must be opened to play back animation (but\
> that’s what we asked for in using clips!), and also in extra overhead in\
> [resolving attribute values](https://openusd.org/release/glossary.html#usdglossary-valueresolution) in the\
> presence of clips. Clips are the reason that\
> [UsdProperty::GetPropertyStack](https://openusd.org/release/api/class_usd_property.html#a2159d3d651cd66e4fb1c724be90ed5e0) requires a timeCode\
> argument, because the set of layers that contribute to an attribute’s\
> value can change over time when it is affected by clips.\
\
Note\
\
For performance and scalability reasons, a UsdStage will ignore any\
composition arcs contained in a “clip” USD file, which means that clips can\
only _usefully_ contain direct (local) opinions about the attributes they\
wish to modify. For more information on value clip behavior and how clips\
are encoded, see [Sequenceable, Re-timeable Animated Value Clips](https://openusd.org/release/api/_usd__page__value_clips.html) in the USD Manual.\
\
## [Value Resolution](https://openusd.org/release/glossary.html\#id445) [](https://openusd.org/release/glossary.html\#usdglossary-valueresolution "Permalink to this heading")\
\
_Value Resolution_ is the algorithm by which final values for [properties](https://openusd.org/release/glossary.html#usdglossary-property) or [metadata](https://openusd.org/release/glossary.html#usdglossary-metadata) are “composed”\
from all of the various [PropertySpecs](https://openusd.org/release/glossary.html#usdglossary-propertyspec) or\
[PrimSpecs](https://openusd.org/release/glossary.html#usdglossary-primspec) that contain data for the property or\
metadatum. Even though value resolution is the act of composing potentially\
many pieces of data together to produce a single value, we distinguish _value_\
_resolution_ from [composition](https://openusd.org/release/glossary.html#usdglossary-composition) because\
understanding the differences between the two aids in effective construction\
and use of USD:\
\
> - **Composition is cached, value resolution is not**\
>\
> The “indexing” performed by the composition algorithm when a [Stage](https://openusd.org/release/glossary.html#usdglossary-stage) is opened, [prims are loaded](https://openusd.org/release/glossary.html#usdglossary-load-unload), or new scene description is authored, is\
> cached _at the prim-level_ for fast access. The USD core does not,\
> however, pre-compute or cache any per-composed-property information, which\
> is a principal design decision aimed at keeping latency low for\
> random-access to composed data, and keeping the minimal memory footprint\
> for USD low. Instead, for attribute value resolution, the USD core\
> provides opt-in facilities such as [UsdAttributeQuery](https://openusd.org/release/api/class_usd_attribute_query.html) and\
> [UsdResolveInfo](https://openusd.org/release/api/class_usd_resolve_info.html), objects a client can construct and\
> retain themselves that cache information that can make repeated value\
> queries faster.\
>\
> - **Composition is internally multi-threaded, value resolution is meant to**\
> **be client multi-threaded.**\
>\
> Composition of a large stage can be a big computation, and USD strives to\
> effectively, internally multi-thread the computation; therefore clients\
> should realize they are unlikely to gain performance from opening multiple\
> stages simultaneously in different threads. Value resolution, however, is\
> a much more lightweight process (moreso for attributes than\
> relationships), and USD’s primary guidance for clients wishing to maximize\
> USD’s performance on multi-core systems is to perform as much simultaneous\
> value resolution and data extraction as possible; USD’s design was guided\
> by this access pattern, and we continue to work on eliminating remaining\
> impediments to efficient multi-threaded value resolution.\
>\
> - **Composition rules vary by composition arc, value resolution rules vary**\
> **by metadatum.**\
>\
> The composition algorithm is the process of interpreting a collection of\
> composition arcs into an “ [index](https://openusd.org/release/glossary.html#usdglossary-index)” of\
> data-containing sites for each composed prim. Value resolution simply\
> consumes the ordered (strong-to-weak) list of contributing sites, and is\
> otherwise insensitive to the particular set of composition arcs that\
> produced that list; but how the data in those sites is combined depends on\
> the particular metadatum being resolved.\
\
**Resolving Metadata**\
\
The basic rule for the metadata value resolution provided by\
[UsdObject::GetMetadata](https://openusd.org/release/api/class_usd_object.html#ad2d76f736733f3ad51aa1ea1df6f4d44) is: **strongest**\
**opinion wins.** Certain metadata such as prim [specifier](https://openusd.org/release/glossary.html#usdglossary-specifier), attribute typeName, and several others have special\
resolution rules; the only one we will discuss here is **dictionary-valued**\
**metadata**, because it is user-facing, as, for example, the [customData\\
dictionary](https://openusd.org/release/api/class_usd_object.html#abea26b9c6a71883a2d9da4ff64952391) authorable on any prim or\
property. Dictionaries are resolved element-wise, so that\
customData\[“keyOne”\] authored on a referencing prim will not block, but\
rather compose into a single dictionary with customData\[“keyTwo”\] on the\
referenced prim.\
\
**Resolving Relationships**\
\
Because relationship targets are [list edited](https://openusd.org/release/glossary.html#usdglossary-listediting), we\
must, in general, combine _all_ of the opinions about the relationship’s\
targets, not just the strongest. The rules for how the opinions combine (in\
weak-to-strong order) are contained inside [SdfListOp::ApplyOperations](https://openusd.org/release/api/class_sdf_list_op.html#ac53720d6b0247734b7d5a313a712a09e).\
\
**Resolving Attributes**\
\
Value resolution for attributes, as performed by [UsdAttribute::Get](https://openusd.org/release/api/class_usd_attribute.html#a9d41bc223be86408ba7d7f74df7c35a9), is\
unique in three ways:\
\
> 1. **Time Offsets**\
>\
> [UsdAttribute::Get](https://openusd.org/release/api/class_usd_attribute.html#a9d41bc223be86408ba7d7f74df7c35a9) is a function of time, so all queries except\
> those evaluated at [UsdTimeCode::Default](https://openusd.org/release/api/class_usd_time_code.html#a8a2192045dc22e90fe08ef2d8d68f3b8) are affected by\
> time-scaling operators such as [Layer Offsets](https://openusd.org/release/glossary.html#usdglossary-layeroffset).\
>\
> 2. **Interpolation**\
>\
> If the requested time coordinate falls between two samples, and the\
> [stage is configured for linear interpolation](https://openusd.org/release/api/class_usd_stage.html#ad29a9aaba12c36407936a21abf514ea4) (which it\
> is by default), then we will [attempt to apply linear interpolation of\\
> the bracketing timeSamples](https://openusd.org/release/api/class_usd_attribute.html#Usd_AttributeInterpolation), before\
> falling back to holding the earlier of the two timeSamples.\
>\
> 3. **Three value sources for each site**\
>\
> For each site in a prim’s Index that may affect a metadatum or\
> relationship, there is just a single place to look for a value - if none\
> is found, we move on to the next site looking for values. For attributes,\
> however, we must examine **three** possible sources for a value for each\
> site, before moving on to the next site in strong-to-weak order:\
>\
> 1. [Value Clips](https://openusd.org/release/glossary.html#usdglossary-valueclips) that are anchored at the site\
>       or an ancestor site in namespace. If no clips are found, or if clips\
>       do not provide a value for the attribute, then…\
>\
> 2. [TimeSamples](https://openusd.org/release/glossary.html#usdglossary-timesample) authored directly at the\
>       site. If there are no TimeSamples, then…\
>\
> 3. [Default Value](https://openusd.org/release/glossary.html#usdglossary-defaultvalue) authored directly at the\
>       site\
\
Effective use of UsdAttribute::Get()\
\
The default UsdTimeCode value for UsdAttribute::Get() is\
UsdTimeCode::Default(), which is almost always a poor choice when\
resolving values on a stage that contains animation. When writing code that\
extracts attribute values from a stage, if the codesite is not provided an\
explicit querying time, you should use UsdTimeCode::EarliestTime(),\
which will ensure that if there is _any_ timeSample authored for the\
attribute, it will provide the value, rather than the _default_, which is all\
that is consulted when UsdTimeCode::Default() is the given time\
coordinate.\
\
## [Variability](https://openusd.org/release/glossary.html\#id446) [](https://openusd.org/release/glossary.html\#usdglossary-variability "Permalink to this heading")\
\
[Attributes](https://openusd.org/release/glossary.html#usdglossary-attribute) possess a special piece of [metadata](https://openusd.org/release/glossary.html#usdglossary-metadata) called _variability_ that serves as a statement of\
intent (typically by a [schema](https://openusd.org/release/glossary.html#usdglossary-schema)) of whether the\
attribute’s value should have [timeSamples](https://openusd.org/release/glossary.html#usdglossary-timesample) that can\
vary its value over time, or whether it should be restricted to having only a\
[default value](https://openusd.org/release/glossary.html#usdglossary-defaultvalue). Variability can have two values:\
varying and uniform; by default, a newly created attribute is\
varying (unless you explicitly specify otherwise), and varying attributes appear\
“unadorned” in the usda text format. Attributes that are uniform, however, will\
appear with the “uniform” modifier in any layer that contains a\
properly-authored opinion for the attribute.\
\
Variability is not consulted or consumed by the core during authoring or value\
resolution, in order to keep those operations fast. It appears in\
[schema-generated documentation](https://openusd.org/release/api/class_usd_geom_mesh.html#a01c7ff0dc2e9b6be9f09db6cfafb7c0a),\
and can be used for validation by higher-level authoring code, and as a hint\
to clients that the value is not expected to change over time. See also\
[UsdAttribute::GetVariability](https://openusd.org/release/api/class_usd_attribute.html#a79655ab3c82828093c685a1582d5e4bc)\
\
usda of the uniform attribute “subdivisionScheme” in the Mesh schema [](https://openusd.org/release/glossary.html#id360 "Permalink to this code")\
\
```\
def Mesh "SimulatableGarment"\
{\
    uniform token subdivisionScheme = "loop"\
}\
\
```\
\
## [Variant](https://openusd.org/release/glossary.html\#id447) [](https://openusd.org/release/glossary.html\#usdglossary-variant "Permalink to this heading")\
\
A _variant_ represents a single, named variation of a [VariantSet](https://openusd.org/release/glossary.html#usdglossary-variantset); each VariantSet can have zero or a single\
**variant selection** expressed in scene description or [as a fallback in a\\
plugin or specified by an application](https://openusd.org/release/api/class_usd_stage.html#Usd_variantManagement), if\
no selection was authored. “variants” is also the keyword in the usda text\
syntax for expressing variant selections, as strings. The following valid usda\
expresses a selection of “High” for the variantSet named\
“lodVariant”. Variant selections are always strings that refer to\
variant names, which can be [USD identifiers](https://openusd.org/release/api/group__group__tf___string.html#gaa129b294af3f68d01477d430b70d40c8)\
with the addition of allowing the “-” character, and an optional leading “.”.\
\
Minimal scene description for expressing a variant selection [](https://openusd.org/release/glossary.html#id361 "Permalink to this code")\
\
```\
#usda 1.0\
\
over "Model" (\
    variants = {\
        string lodVariant = "High"\
    }\
)\
{\
}\
\
```\
\
A variant can contain overriding opinions (for properties, metadata, and more),\
as well as any arbitrary scene description (entire child prim subtrees, etc).\
Variants can also **include additional composition arcs**. This gives us great\
flexibility in building up variations out of existing, modular pieces that\
can be [referenced](https://openusd.org/release/glossary.html#usdglossary-references), [inherited](https://openusd.org/release/glossary.html#usdglossary-inherits), etc. In the following example snippet, the\
“referenceVariantSet” VariantSet contains two variants that reference different\
USD assets. Changing the variant selection controls which asset is referenced in\
the scene.\
\
VariantSet with references [](https://openusd.org/release/glossary.html#id362 "Permalink to this code")\
\
```\
over "Model" (\
    prepend variantSets = "referenceVariantSet"\
    variants = {\
       string referenceVariantSet = "asset1"\
    }\
)\
{\
    variantSet "referenceVariantSet" = {\
        "asset1" (\
            prepend references = @Asset1.usda@\
        ) {\
        }\
        "asset2" (\
            prepend references = @Asset2.usda@\
        ) {\
        }\
    }\
}\
\
```\
\
## [VariantSet](https://openusd.org/release/glossary.html\#id448) [](https://openusd.org/release/glossary.html\#usdglossary-variantset "Permalink to this heading")\
\
A _VariantSet_ is a [composition arc](https://openusd.org/release/glossary.html#usdglossary-compositionarcs) that\
allows a content creator to package a discrete set of alternatives, between\
which a downstream consumer is able to non-destructively switch, or augment. A\
reasonable way to think about VariantSets is as a “switchable reference”. Each\
[Variant](https://openusd.org/release/glossary.html#usdglossary-variant) of a VariantSet encapsulates a tree of scene\
description that will be composed onto the prim on which the VariantSet is\
defined, when the Variant is selected. VariantSet names must be\
[legal USD identifiers](https://openusd.org/release/api/group__group__tf___string.html#gaa129b294af3f68d01477d430b70d40c8).\
Here is an example of a very simple VariantSet:\
\
simpleVariantSet.usd [](https://openusd.org/release/glossary.html#id363 "Permalink to this code")\
\
```\
#usda 1.0\
\
def Xform "Implicits" (\
    append variantSets = "shapeVariant"\
)\
{\
    variantSet "shapeVariant" = {\
        "Capsule" {\
            def Capsule "Pill"\
            {\
            }\
        }\
        "Cone" {\
            def Cone "PartyHat"\
            {\
            }\
        }\
        "Cube" {\
            def Cube "Box"\
            {\
            }\
        }\
        "Cylinder" {\
            def Cylinder "Tube"\
            {\
            }\
        }\
        "Sphere" {\
            def Sphere "Ball"\
            {\
            }\
        }\
    }\
}\
\
```\
\
**Things to note about simpleVariantSet.usd**\
\
> - **Variant selections are not required.**\
>\
> If you copy/paste, then open simpleVariantSet.usd in\
> [usdview](https://openusd.org/release/toolset.html#usdview), nothing will be drawn! This is because we have not\
> specified a selection for shapeVariant (see [Variant](https://openusd.org/release/glossary.html#usdglossary-variant) for what a selection looks like), which is always\
> a valid thing to do, and simply means none of the variants contained in\
> the VariantSet will be applied. If you select the prim\
> /Implicits in **usdview's** browser, and open the\
> “Metadata” tab of the inspector in the lower-right-hand corner, you will\
> find a drop-down selection box for shapeVariant, from which you\
> can select any of the variations and see the viewport update in\
> response. Each time you make a selection, usdview is authoring a variant\
> selection on /Implicits in the opened stage’s [session layer](https://openusd.org/release/glossary.html#usdglossary-sessionlayer).\
>\
> - **Variants can contain \*any\* scene description.**\
>\
> Variants are not just about overrides! In\
> simpleVariantSet.usd, we are creating a\
> differently-named-and-typed child prim in each variant. Each variant can\
> in fact create an entire subtree of prims (that will be combined on top of\
> any referenced scene description), as well as apply overrides.\
>\
> - **VariantSets are “inlined” in usda.**\
>\
> Although VariantSets, in composition terms, behave very much like\
> references and other arcs that target “remote” scene description,\
> VariantSets are presented as “inlined scene description” inside the prim\
> they modify (or rather, inside the prim, the root of whose subtree the\
> VariantSet modifies).\
\
**Characteristics of VariantSets**\
\
> - **A prim can have an unlimited number of VariantSets** declared directly\
> on itself; the VariantSets can be ordered (initial order is authored\
> order) as part of their [List Editing](https://openusd.org/release/glossary.html#usdglossary-listediting) nature,\
> and the final order of the VariantSets provides their relative strength\
> with respect to each other, should their opinions overlap.\
>\
> - Because VariantSets are just composition arcs, this implies that\
> **VariantSets can be directly nested inside each other**, by allowing a\
> Variant of a VariantSet to introduce a new VariantSet. This allows a\
> concise encoding for dependent VariantSets, wherein the options/variants\
> available for the inner/nested VariantSet depends on the selection of the\
> outer VariantSet; [see below for an example](https://openusd.org/release/glossary.html#usdglossary-nestedvariants).\
>\
> - Higher-level/downstream contexts can **dynamically add new Variants to**\
> **existing VariantSets.** This ability facilitates the use of VariantSets to\
> provide as-you-go asset version control, which can be especially useful,\
> for example, in refining simulations or special effects. We might\
> reference in a low-res water simulation in a weak “sequence layer” that is\
> shared by a number of shots in a sequence. One shot , however, requires a\
> closeup, necessitating a higher resolution simulation. If the\
> sequence-level simulation was placed inside a version VariantSet,\
> in a SequenceBase Variant, then the special shot can, in a layer\
> stronger, simply introduce a new CloseupShot Variant to the\
> version VariantSet. In that shot, both variants will be available\
> and selectable. In other shots, only the SequenceBase Variant will\
> be available.\
>\
> - **VariantSets allow optimal scattering of instancing variation**, because\
> for any given [instanceable prim](https://openusd.org/release/glossary.html#usdglossary-instanceable), its\
> instancing key, which is what we use to decide which [scene-level\\
> instances](https://openusd.org/release/glossary.html#usdglossary-instancing) will share the same prototype, takes\
> the variant selections on the prim into account. So, if you build and\
> publish an asset with one-or-more VariantSets on its root prim, an\
> environment-building tool can confidently add many instances of the asset\
> to a scene, scattering variation by scattering variant-selections, and the\
> USD core will ensure only as many unique prototypes are compose as there\
> are unique combinations of variant-selections.\
\
**Nested VariantSets**\
\
As mentioned above, VariantSets can be nested directly inside each other, on the\
same prim. VariantSet nesting in USD can be accomplished straightforwardly by\
nesting the use of [UsdEditContext](https://openusd.org/release/api/class_usd_edit_context.html) objects in code. Following is a\
small python program that nests two VariantSets, demonstrating that the contents\
of the inner VariantSet can vary from Variant to Variant of the outer\
VariantSet.\
\
nestedVariants.py [](https://openusd.org/release/glossary.html#id364 "Permalink to this code")\
\
```\
from pxr import Sdf, Usd\
stage = Usd.Stage.CreateNew("nestedVariants.usd")\
prim = stage.DefinePrim("/Employee")\
title = prim.CreateAttribute("title", Sdf.ValueTypeNames.String)\
variantSets = prim.GetVariantSets()\
\
critters = [ "Bug", "Bear", "Dragon" ]\
jobs = [ "Squasher", "Rider", "Trainer" ]\
\
critterVS = variantSets.AppendVariantSet("critterVariant")\
for critter in critters:\
    critterVS.AppendVariant(critter)\
    critterVS.SetVariantSelection(critter)\
    with critterVS.GetVariantEditContext():\
        # All edits now go "inside" the selected critter variant\
        jobVS = variantSets.AppendVariantSet("jobVariant")\
        for job in jobs:\
            if (job != "Squasher" or critter == "Bug") and \\
               (job != "Rider" or critter != "Bug") :\
                jobVS.AppendVariant(job)\
                jobVS.SetVariantSelection(job)\
                with jobVS.GetVariantEditContext():\
                    # Now edits *additionally* go inside the selected job variant\
                    title.Set(critter + job)\
\
stage.GetRootLayer().Save()\
\
```\
\
Try loading the resulting nestedVariants.usd in **usdview**.\
Note that as you select different _critterVariants_, the contents of\
_jobVariant_ will change, as will of course, the resolved value of the\
title attribute on the /Employee prim. If you select “Bug”\
and “Squasher”, respectively, and then change critterVariant to\
Bear or Dragon” the jobVariant selection will become\
empty, because the selected variant is not a valid selection for the\
jobVariant VariantSet defined inside those critter variants. This is not\
an error condition, and the result is simply that no jobVariant is\
applied.\
\
## [Visibility](https://openusd.org/release/glossary.html\#id449) [](https://openusd.org/release/glossary.html\#usdglossary-visibility "Permalink to this heading")\
\
_Visibility_ is a builtin attribute of the [UsdGeomImageable](https://openusd.org/release/api/class_usd_geom_imageable.html) base\
schema, and models the simplest form of “pruning” invisibility that is\
supported by most DCC apps. Visibility can take on two possible token string\
values:\
\
> - **inherited** (the fallback value if the attribute is not authored)\
>\
> - **invisible**\
\
If the [resolved value](https://openusd.org/release/glossary.html#usdglossary-valueresolution) is **invisible**, then\
neither the prim itself nor any prims in the subtree rooted at the prim should\
be rendered - this is what we mean by “pruning invisibility”, since invisible\
subtrees are definitively pruned in their entirety. If the resolve value is\
**inherited**, it means that the _computed visibility_ (as provided by\
[UsdGeomImageable::ComputeVisibility](https://openusd.org/release/api/class_usd_geom_imageable.html#ac0bfa1554d9b57303c14f71d084c578d)) of the prim will be whatever the\
computed value of the prim’s namespace parent is.\
\
Visibility may be animated, allowing a sub-tree of geometry to be renderable for\
some segment of a shot, and absent from others; unlike the action of\
[deactivating](https://openusd.org/release/glossary.html#usdglossary-active-inactive) geometry prims, invisible\
geometry is still available for inspection, for positioning, for defining\
volumes, etc.
