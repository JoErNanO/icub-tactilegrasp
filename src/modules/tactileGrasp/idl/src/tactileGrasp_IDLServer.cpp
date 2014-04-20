// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <tactileGrasp_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class tactileGrasp_IDLServer_open : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("open",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class tactileGrasp_IDLServer_grasp : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("grasp",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class tactileGrasp_IDLServer_crush : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("crush",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class tactileGrasp_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("quit",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class tactileGrasp_IDLServer_setThreshold : public yarp::os::Portable {
public:
  int32_t aFinger;
  double aThreshold;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("setThreshold",1,1)) return false;
    if (!writer.writeI32(aFinger)) return false;
    if (!writer.writeDouble(aThreshold)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

bool tactileGrasp_IDLServer::open() {
  bool _return = false;
  tactileGrasp_IDLServer_open helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tactileGrasp_IDLServer::open()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tactileGrasp_IDLServer::grasp() {
  bool _return = false;
  tactileGrasp_IDLServer_grasp helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tactileGrasp_IDLServer::grasp()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tactileGrasp_IDLServer::crush() {
  bool _return = false;
  tactileGrasp_IDLServer_crush helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tactileGrasp_IDLServer::crush()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tactileGrasp_IDLServer::quit() {
  bool _return = false;
  tactileGrasp_IDLServer_quit helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tactileGrasp_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tactileGrasp_IDLServer::setThreshold(const int32_t aFinger, const double aThreshold) {
  bool _return = false;
  tactileGrasp_IDLServer_setThreshold helper;
  helper.aFinger = aFinger;
  helper.aThreshold = aThreshold;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tactileGrasp_IDLServer::setThreshold(const int32_t aFinger, const double aThreshold)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool tactileGrasp_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "open") {
      bool _return;
      _return = open();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "grasp") {
      bool _return;
      _return = grasp();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "crush") {
      bool _return;
      _return = crush();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setThreshold") {
      int32_t aFinger;
      double aThreshold;
      if (!reader.readI32(aFinger)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(aThreshold)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setThreshold(aFinger,aThreshold);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> tactileGrasp_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("open");
    helpString.push_back("grasp");
    helpString.push_back("crush");
    helpString.push_back("quit");
    helpString.push_back("setThreshold");
    helpString.push_back("help");
  }
  else {
    if (functionName=="open") {
      helpString.push_back("bool open() ");
      helpString.push_back("Opens the robot hand. ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="grasp") {
      helpString.push_back("bool grasp() ");
      helpString.push_back("Grasp an object using feedback from the fingertips tactile sensors. ");
      helpString.push_back("The grasping movement is stopped upon contact detection. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="crush") {
      helpString.push_back("bool crush() ");
      helpString.push_back("Grasp object without using the feedback from the fingertips tactile sensors. ");
      helpString.push_back("The grasping movement is not controlled and the object is therefore crushed. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("Quit the module. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setThreshold") {
      helpString.push_back("bool setThreshold(const int32_t aFinger, const double aThreshold) ");
      helpString.push_back("Set the touch threshold. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


