#ifndef _PLUGIN_HH_
#define _PLUGIN_HH_

class PluginAbstract {
public:
  PluginAbstract(){};
  virtual ~PluginAbstract(){};
  virtual void Initialization(std::string &astr) = 0;
};

typedef PluginAbstract *createPlugin_t();
#endif /* _PLUGIN_HH_ */
