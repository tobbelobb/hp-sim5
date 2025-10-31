#include <ObjectModel/ObjectModel.h>

#include <GCodes/GCodeException.h>

#include <algorithm>
#include <cstring>

namespace
{
constexpr std::size_t ClampIndex(std::size_t value)
{
	return (value < MaxExpressionArrayIndices) ? value : (MaxExpressionArrayIndices - 1);
}
}

ObjectExplorationContext::ObjectExplorationContext(const GCodeBuffer* gbp, bool wal, const char*, unsigned int initialMaxDepth, size_t initialBufferOffset) noexcept
	: startMillis(0), initialBufOffset(initialBufferOffset), maxDepth(initialMaxDepth), currentDepth(0), startElement(0),
	  nextElement(-1), numIndicesProvided(0), numIndicesCounted(0), line(-1), column(-1), gb(gbp), shortForm(false),
	  wantArrayLength(wal), wantExists(false), includeNonLive(false), includeImportant(false), includeNulls(false),
	  obsoleteFieldQueried(false), excludedFlags(ObjectModelEntryFlags::none)
{
	std::fill(std::begin(indices), std::end(indices), 0);
}

ObjectExplorationContext::ObjectExplorationContext(const GCodeBuffer* gbp, bool wal, bool wex, int p_line, int p_col) noexcept
	: startMillis(0), initialBufOffset(0), maxDepth(16), currentDepth(0), startElement(0), nextElement(-1),
	  numIndicesProvided(0), numIndicesCounted(0), line(p_line), column(p_col), gb(gbp), shortForm(false),
	  wantArrayLength(wal), wantExists(wex), includeNonLive(false), includeImportant(false), includeNulls(false),
	  obsoleteFieldQueried(false), excludedFlags(ObjectModelEntryFlags::none)
{
	std::fill(std::begin(indices), std::end(indices), 0);
}

ObjectExplorationContext::ObjectExplorationContext() noexcept
	: ObjectExplorationContext(nullptr, false, nullptr, 16, 0)
{
}

void ObjectExplorationContext::AddIndex(int32_t index) THROWS(GCodeException)
{
	if (numIndicesCounted >= MaxExpressionArrayIndices)
	{
		throw GCodeException("too many indices");
	}
	indices[numIndicesCounted++] = index;
}

void ObjectExplorationContext::AddIndex() THROWS(GCodeException)
{
	AddIndex(0);
}

void ObjectExplorationContext::RemoveIndex() THROWS(GCodeException)
{
	if (numIndicesCounted == 0)
	{
		throw GCodeException("no index to remove");
	}
	--numIndicesCounted;
}

void ObjectExplorationContext::ProvideIndex(int32_t index) THROWS(GCodeException)
{
	const std::size_t pos = ClampIndex(numIndicesProvided);
	indices[pos] = index;
	if (numIndicesProvided < MaxExpressionArrayIndices)
	{
		++numIndicesProvided;
	}
}

int32_t ObjectExplorationContext::GetIndex(std::size_t n) const noexcept
{
	if (n >= numIndicesProvided)
	{
		return 0;
	}
	return indices[n];
}

int32_t ObjectExplorationContext::GetLastIndex() const noexcept
{
	return (numIndicesProvided == 0) ? 0 : indices[numIndicesProvided - 1];
}

bool ObjectExplorationContext::ShouldReport(const ObjectModelEntryFlags f) const noexcept
{
	return ((static_cast<uint8_t>(f) & static_cast<uint8_t>(excludedFlags)) == 0);
}

GCodeException ObjectExplorationContext::ConstructParseException(const char* msg) const noexcept
{
	return GCodeException(line, column, msg);
}

GCodeException ObjectExplorationContext::ConstructParseException(const char* msg, const char* sparam) const noexcept
{
	return GCodeException(line, column, msg, sparam);
}

void ObjectExplorationContext::CheckStack(uint32_t) const THROWS(GCodeException)
{
}

