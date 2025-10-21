#include <ObjectModel/Variable.h>

#include <General/StringRef.h>

#include <cctype>
#include <cstring>
#include <memory>

Variable::Variable(const char* str, std::size_t strLen, ExpressionValue& pVal, int16_t pScope)
	: name(str, strLen),
	  val(pVal),
	  scope(pScope)
{
}

Variable::~Variable() = default;

bool Variable::IsValidVariableName(const char* str) noexcept
{
	if (str == nullptr || *str == '\0')
	{
		return false;
	}
	if (!std::isalpha(static_cast<unsigned char>(*str)) && *str != '_')
	{
		return false;
	}
	for (const char* p = str + 1; *p != '\0'; ++p)
	{
		if (!std::isalnum(static_cast<unsigned char>(*p)) && *p != '_')
		{
			return false;
		}
	}
	return true;
}

void Variable::Assign(ExpressionValue& ev)
{
	val = ev;
}

void Variable::AssignIndexed(const ExpressionValue& ev, std::size_t, const uint32_t*)
{
	ExpressionValue copy(ev);
	val = copy;
}

void Variable::AssignArray(std::size_t numElements, function_ref<ExpressionValue(std::size_t)> producer) noexcept
{
	ArrayHandle ah;
	if (numElements != 0)
	{
		ah.Allocate(numElements);
		for (std::size_t i = 0; i < numElements; ++i)
		{
			ExpressionValue elem = producer(i);
			ah.AssignElement(i, elem);
		}
	}
	val.SetArrayHandle(ah);
}

VariableSet::~VariableSet()
{
	Clear();
}

void VariableSet::AssignFrom(VariableSet& other) noexcept
{
	Clear();
	other.IterateWhile([this](unsigned int, const Variable& v) noexcept -> bool
	{
		ExpressionValue copy = v.GetValue();
		(void)InsertNew(v.GetName().Ptr(), v.GetValue(), v.GetScope());
		return true;
	});
}

Variable* VariableSet::Lookup(const char* str, bool wantParameter) noexcept
{
	for (LinkedVariable* current = root; current != nullptr; current = current->next)
	{
		const auto namePtr = current->v.GetName();
		if (namePtr.Ptr() != nullptr && std::strcmp(namePtr.Ptr(), str) == 0)
		{
			if (!wantParameter || current->v.GetScope() < 0)
			{
				return &current->v;
			}
		}
	}
	return nullptr;
}

const Variable* VariableSet::Lookup(const char* str, std::size_t length, bool wantParameter) const noexcept
{
	for (LinkedVariable* current = root; current != nullptr; current = current->next)
	{
		const auto namePtr = current->v.GetName();
		if (namePtr.Ptr() != nullptr && std::strncmp(namePtr.Ptr(), str, length) == 0 && namePtr.Ptr()[length] == '\0')
		{
			if (!wantParameter || current->v.GetScope() < 0)
			{
				return &current->v;
			}
		}
	}
	return nullptr;
}

Variable* VariableSet::InsertNew(const char* str, ExpressionValue pVal, int16_t pScope)
{
	return InsertNew(str, std::strlen(str), pVal, pScope);
}

Variable* VariableSet::InsertNew(const char* str, std::size_t strLen, ExpressionValue pVal, int16_t pScope)
{
	auto* node = new LinkedVariable(str, strLen, pVal, pScope, root);
	root = node;
	return &node->v;
}

void VariableSet::EndScope(uint8_t blockNesting) noexcept
{
	LinkedVariable* prev = nullptr;
	for (LinkedVariable* current = root; current != nullptr; )
	{
		if (current->v.GetScope() >= 0 && static_cast<uint8_t>(current->v.GetScope()) >= blockNesting)
		{
			LinkedVariable* toDelete = current;
			current = current->next;
			if (prev == nullptr)
			{
				root = current;
			}
			else
			{
				prev->next = current;
			}
			delete toDelete;
		}
		else
		{
			prev = current;
			current = current->next;
		}
	}
}

void VariableSet::Delete(const char* str) noexcept
{
	LinkedVariable* prev = nullptr;
	for (LinkedVariable* current = root; current != nullptr; )
	{
		const auto namePtr = current->v.GetName();
		if (namePtr.Ptr() != nullptr && std::strcmp(namePtr.Ptr(), str) == 0)
		{
			LinkedVariable* victim = current;
			current = current->next;
			if (prev == nullptr)
			{
				root = current;
			}
			else
			{
				prev->next = current;
			}
			delete victim;
		}
		else
		{
			prev = current;
			current = current->next;
		}
	}
}

void VariableSet::Clear() noexcept
{
	while (root != nullptr)
	{
		LinkedVariable* next = root->next;
		delete root;
		root = next;
	}
}

void VariableSet::IterateWhile(function_ref_noexcept<bool(unsigned int, const Variable&) noexcept> func) const noexcept
{
	unsigned int index = 0;
	for (LinkedVariable* current = root; current != nullptr; current = current->next, ++index)
	{
		if (!func(index, current->v))
		{
			break;
		}
	}
}

