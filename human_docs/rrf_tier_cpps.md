# Tier 1: Foundational Utilities and Data Structures

Platform/ArrayHandle.cpp
Platform/StringHandle.cpp
Platform/OutputMemory.cpp
ObjectModel/Variable.cpp
ObjectModel/ObjectModel.cpp
ObjectModel/GlobalVariables.cpp

# Tier 2: Core Machine Components (Stateful but not Active)

Tools/Filament.cpp
Tools/Tool.cpp
Movement/Kinematics/Kinematics.cpp
Movement/Kinematics/CoreKinematics.cpp
HangprinterKinematics.cpp

# Tier 3: The Motion Planning Engine (Bottom-Up)
Movement/DriveMovement.cpp
Movement/DDA.cpp
Movement/DDARing.cpp
Movement/Move.cpp

# Tier 4: G-Code Interpretation and High-Level Logic
GCodes/GCodeBuffer/GCodeBuffer.cpp
ExpressionParser.cpp
StringParser.cpp
GCodes/GCodeMachineState.cpp
GCodes/GCodeInput.cpp
GCodes/GCodeQueue.cpp
GCodes/GCodes.cpp
GCodes2.cpp
GCodes3.cpp

# Tier 5: Top-Level Application Objects
PrintMonitor/PrintMonitor.cpp
Platform/RepRap.cpp

Your own main.cpp
 - Instantiate your fake platform and hardware classes.
 - Instantiate the RepRap object.
 - Call the initialization sequences.
 - Contain the main loop that reads G-code from a file and feeds it to the GCodes module.
