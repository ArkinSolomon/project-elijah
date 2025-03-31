#pragma once
#include "enum_type.h"
#include "elijah_state_framework.h"

namespace elijah_state_framework
{
  FRAMEWORK_TEMPLATE_DECL
  class ReliableComponentHelper
  {
  public:
    ReliableComponentHelper(ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, EFaultKey fault_key);
    virtual ~ReliableComponentHelper() = default;

    void update(TStateData& state);

    [[nodiscard]] bool is_connected() const;

  protected:
    virtual std::string on_init(TStateData& state) = 0;
    virtual std::string on_update(TStateData& state) = 0;

    [[nodiscard]] ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* get_framework() const;

  private:
    EFaultKey fault_key;
    ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework;

    bool connected = false;
  };
}

FRAMEWORK_TEMPLATE_DECL
elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>::ReliableComponentHelper(
  ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
  EFaultKey fault_key) : fault_key(fault_key), framework(framework)
{
}

FRAMEWORK_TEMPLATE_DECL
elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* elijah_state_framework::ReliableComponentHelper<
  FRAMEWORK_TEMPLATE_TYPES>::get_framework() const
{
  return framework;
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>::update(TStateData& state)
{
  connected = connected && !framework->is_faulted(fault_key);

  if (!connected)
  {
    const std::string ret_message = on_init(state);

    if (!ret_message.empty())
    {
      framework->set_fault(fault_key, true, ret_message);
      return;
    }
    connected = true;
  }

  const std::string ret_message = on_update(state);
  if (!ret_message.empty())
  {
    framework->set_fault(fault_key, true, ret_message);
    connected = false;
    return;
  }

  framework->set_fault(fault_key, false, ret_message);
}

FRAMEWORK_TEMPLATE_DECL
bool elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>::is_connected() const
{
  return connected;
}
