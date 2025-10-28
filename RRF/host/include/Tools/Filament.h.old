#pragma once

#include <cstddef>
#include <string>

inline constexpr size_t FilamentNameLength = 32;

class Filament
{
public:
	explicit Filament(int extr) noexcept : extruder(extr) {}

	int GetExtruder() const noexcept { return extruder; }
	const char* GetName() const noexcept { return filamentName.c_str(); }

	bool IsLoaded() const noexcept { return !filamentName.empty(); }
	void Load(const char* name) noexcept
	{
		filamentName = (name != nullptr) ? name : "";
	}
	void Unload() noexcept { filamentName.clear(); }

	void LoadAssignment() noexcept {}

	static void SaveAssignments() noexcept {}
	static Filament* GetFilamentByExtruder(int) noexcept { return nullptr; }

private:
	int extruder;
	std::string filamentName;
};
