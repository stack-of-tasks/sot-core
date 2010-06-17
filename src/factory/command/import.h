// -*- c++ -*-
#ifndef SOT_FACTORY_COMMAND_IMPORT_H
# define SOT_FACTORY_COMMAND_IMPORT_H
# include <iosfwd>
# include <string>
# include <vector>

class sotInterpretor;

namespace sot
{
  namespace command
  {
    namespace
    {
      extern std::vector<std::string> importPaths;
    } // end of anonymous namespace.

    /// \brief Implement sot interpretor import command.
    ///
    /// The import command sources a file and searches automatically
    /// for it in the importPaths.
    void import (sotInterpretor& interpretor,
		 const std::string& cmdLine,
		 std::istringstream& cmdArg,
		 std::ostream& os);

    /// \brief Implement sot interpretor pushImportPaths command.
    ///
    /// Append a path to importPaths.
    void pushImportPaths (sotInterpretor& interpretor,
			  const std::string& cmdLine,
			  std::istringstream& cmdArg,
			  std::ostream& os);

    /// \brief Implement sot interpretor popImportPaths command.
    ///
    /// Drop the last path of importPaths.
    void popImportPaths (sotInterpretor& interpretor,
			 const std::string& cmdLine,
			 std::istringstream& cmdArg,
			 std::ostream& os);

  } // end of namespace command.
} // end of namespace sot.

#endif //! SOT_FACTORY_COMMAND_IMPORT_H
