#!/usr/bin/env python3
"""
Step 1: Load and inspect the existing USDA scene using USD Python API
"""

from pxr import Usd, UsdGeom

def inspect_scene():
    """Load and inspect the current flipper_scene.usda"""
    
    # Open the existing scene
    stage = Usd.Stage.Open("flipper_scene.usda")
    
    print("=== FLIPPER SCENE INSPECTION ===")
    print(f"Stage root layer: {stage.GetRootLayer().identifier}")
    print()
    
    # Traverse all prims in the scene
    print("=== ALL PRIMS IN SCENE ===")
    for prim in stage.Traverse():
        print(f"Path: {prim.GetPath()}")
        print(f"  Type: {prim.GetTypeName()}")
        print(f"  Properties: {prim.GetPropertyNames()}")
        
        # Check for custom data
        if prim.HasAuthoredMetadata("customData"):
            custom_data = prim.GetMetadata("customData")
            print(f"  Custom Data: {custom_data}")
        
        # Print all attributes and their values
        for prop_name in prim.GetPropertyNames():
            attr = prim.GetAttribute(prop_name)
            if attr:
                try:
                    value = attr.Get()
                    print(f"  {prop_name}: {value}")
                except:
                    print(f"  {prop_name}: <unable to get value>")
        
        print()
    
    # Analyze the scene structure
    print("=== SCENE ANALYSIS ===")
    
    # Get the root prim
    flipper_scene = stage.GetPrimAtPath("/FlipperScene")
    print(f"Root scene prim: {flipper_scene.GetPath()}")
    
    # Categorize prims by purpose
    balls = []
    obstacles = []
    joints = []
    cable_paths = []
    
    for prim in flipper_scene.GetChildren():
        if prim.HasAuthoredMetadata("customData"):
            custom_data = prim.GetMetadata("customData")
            purpose = custom_data.get("purpose", "unknown")
            
            if purpose == "ball":
                balls.append(prim)
            elif purpose == "obstacle":
                obstacles.append(prim)
            elif purpose == "cable_joint":
                joints.append(prim)
            elif purpose == "cable_path":
                cable_paths.append(prim)
    
    print(f"Found {len(balls)} balls: {[p.GetName() for p in balls]}")
    print(f"Found {len(obstacles)} obstacles: {[p.GetName() for p in obstacles]}")
    print(f"Found {len(joints)} cable joints: {[p.GetName() for p in joints]}")
    print(f"Found {len(cable_paths)} cable paths: {[p.GetName() for p in cable_paths]}")
    
    print("\n=== CABLE JOINT ANALYSIS ===")
    for joint in joints:
        print(f"Joint: {joint.GetName()}")
        entity_a = joint.GetAttribute("entityA").Get()
        entity_b = joint.GetAttribute("entityB").Get()
        rest_length = joint.GetAttribute("restLength").Get()
        attach_a = joint.GetAttribute("attachA").Get()
        attach_b = joint.GetAttribute("attachB").Get()
        
        print(f"  EntityA: {entity_a}")
        print(f"  EntityB: {entity_b}")
        print(f"  RestLength: {rest_length}")
        print(f"  AttachA: {attach_a}")
        print(f"  AttachB: {attach_b}")
        print()
    
    print("=== CABLE PATH ANALYSIS ===")
    for path in cable_paths:
        print(f"Cable Path: {path.GetName()}")
        joints_attr = path.GetAttribute("joints").Get()
        link_types = path.GetAttribute("linkTypes").Get()
        cw = path.GetAttribute("cw").Get()
        stored = path.GetAttribute("stored").Get()
        
        print(f"  Joints: {joints_attr}")
        print(f"  Link Types: {link_types}")
        print(f"  CW: {cw}")
        print(f"  Stored: {stored}")
        print()
    
    return stage

if __name__ == "__main__":
    inspect_scene()