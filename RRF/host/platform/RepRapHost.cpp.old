#include <Platform/RepRap.h>

#include <algorithm>
#include <string_view>

namespace
{
constexpr size_t ClampIndex(int32_t rawIndex)
{
	return (rawIndex < 0) ? 0u : static_cast<size_t>(rawIndex);
}

constexpr bool ConsumePrefix(std::string_view& path, std::string_view prefix)
{
	if (path.size() < prefix.size() || path.compare(0, prefix.size(), prefix) != 0)
	{
		return false;
	}
	path.remove_prefix(prefix.size());
	return true;
}

constexpr void ConsumeIfPresent(std::string_view& path, char token)
{
	if (!path.empty() && path.front() == token)
	{
		path.remove_prefix(1);
	}
}

size_t RequestedIndex(const ObjectExplorationContext& context) noexcept
{
	if (context.GetNumIndicesCounted() == 0)
	{
		return 0;
	}
	return ClampIndex(context.GetLastIndex());
}

bool HandleInputsPath(RepRap& reprap, ObjectExplorationContext& context, std::string_view path, ExpressionValue& out)
{
	if (!ConsumePrefix(path, "inputs"))
	{
		return false;
	}

	const size_t numInputs = reprap.GetGCodes().GetNumInputs();

	if (path.empty())
	{
		if (context.WantArrayLength())
		{
			out.SetUnsigned(static_cast<uint32_t>(numInputs));
			return true;
		}
		if (context.WantExists())
		{
			out.SetBool(numInputs > 0);
			return true;
		}
	}

	ConsumeIfPresent(path, '^');
	ConsumeIfPresent(path, '.');

	if (path == "axesRelative")
	{
		const size_t index = RequestedIndex(context);
		const bool exists = (index < numInputs);

		if (context.WantExists())
		{
			out.SetBool(exists);
			return true;
		}

		if (!exists)
		{
			out.SetNull(nullptr);
			return true;
		}

		out.SetBool(reprap.GetGCodes().GetAxesRelative(index));
		return true;
	}

	if (context.WantExists())
	{
		out.SetBool(false);
		return true;
	}

	return false;
}

bool HandleMoveAxesPath(RepRap& reprap, ObjectExplorationContext& context, std::string_view path, ExpressionValue& out)
{
	if (!ConsumePrefix(path, "move"))
	{
		return false;
	}

	ConsumeIfPresent(path, '.');

	if (!ConsumePrefix(path, "axes"))
	{
		return false;
	}

	const size_t axisCount = reprap.GetGCodes().GetAxisCount();

	if (path.empty())
	{
		if (context.WantArrayLength())
		{
			out.SetUnsigned(static_cast<uint32_t>(axisCount));
			return true;
		}
		if (context.WantExists())
		{
			out.SetBool(axisCount > 0);
			return true;
		}
	}

	ConsumeIfPresent(path, '^');
	ConsumeIfPresent(path, '.');

	if (path == "userPosition")
	{
		const size_t axisIndex = RequestedIndex(context);
		const bool exists = (axisIndex < axisCount);

		if (context.WantExists())
		{
			out.SetBool(exists);
			return true;
		}

		if (!exists)
		{
			out.SetNull(nullptr);
			return true;
		}

		out.SetFloat(reprap.GetGCodes().GetUserPosition(axisIndex));
		return true;
	}

	if (context.WantExists())
	{
		out.SetBool(false);
		return true;
	}

	return false;
}
} // namespace

RepRap reprap;

RepRap::RepRap() noexcept
	: platform(),
	  gCodes(),
	  heat(),
	  printMonitor(platform, gCodes),
	  fansManager(),
	  globalVariables()
{
	platform.Init();
	heat.Init();
	fansManager.Init();
	printMonitor.Init();
}

ExpressionValue RepRap::GetObjectValueUsingTableNumber(ObjectExplorationContext& context,
													   const ObjectModelClassDescriptor*,
													   const char* idString,
													   uint8_t) THROWS(GCodeException)
{
	ExpressionValue result;
	result.SetNull(nullptr);

	if (idString != nullptr && *idString != '\0')
	{
		std::string_view path(idString);

		if (HandleInputsPath(*this, context, path, result))
		{
			return result;
		}

		path = idString;
		if (HandleMoveAxesPath(*this, context, path, result))
		{
			return result;
		}

		platform.MessageF(WarningMessage, "object model path '%s' not supported in host build\n", idString);
	}
	else
	{
		platform.Message(GenericMessage, "object model path lookup not supported in host build\n");
	}

	return result;
}
