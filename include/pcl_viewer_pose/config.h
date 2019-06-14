#ifndef CONFIG_H
#define CONFIG_H

#include "pcl_viewer_pose/common.h"

#include <map>
#include <boost/shared_ptr.hpp>

namespace pcl_viewer_pose
{
class Config
{
    private:
        static boost::shared_ptr<Config> config_;
        std::ifstream config_file_;
        std::string delimiter_;
        std::string comment_;
        std::map<std::string, std::string> config_map_;

        typedef std::map<std::string, std::string>::iterator map_iterator;
        typedef std::map<std::string, std::string>::const_iterator map_const_iterator;
        
        Config(){}

    public:
        ~Config();

        static void setConfigFile(const std::string& filename,
                                  std::string delimiter = "=",
                                  std::string comment = "#");

        template< typename T> static T get(const std::string& key);
        template< typename T> static T get(const std::string& key, 
                                           const T& in_value);

        template< typename T> 
        bool getInfo(T& out_var, const std::string& key) const;

        template< typename T>
        bool getInfo(T& out_var, const std::string& key, const T& in_value)const;

        bool ifFileExist(std::string filename);

        bool ifKeyExist(const std::string& key) const;

        template< typename T> void addKey(const std::string& key, const T& in_value);

        void remove(const std::string& key);

        std::string getDelimiter() const {return delimiter_;}
        std::string getComment() const {return comment_;}

        std::string setDelimiter(const std::string& delimiter)
        {
            std::string old_delimiter = delimiter_;
            delimiter_ = delimiter;
            return old_delimiter;
        }

        std::string setComment(const std::string& comment)
        {
            std::string old_comment = comment_;
            comment_ = comment;
            return old_comment;
        }

    protected:
        template<typename T> static std::string T_to_string(const T& t);

        template<typename T> static T string_to_T(const std::string& s);

        static void trim(std::string& s);

    // Exceptions
    public:
        struct FILE_NOT_FOUND
        {
            std::string filename;
            FILE_NOT_FOUND(const std::string& filename_ = std::string()):
                filename(filename_){}
        };
        struct KEY_NOT_FOUND
        {
            std::string key;
            KEY_NOT_FOUND(const std::string& key_ = std::string()):
                key(key_){};
        };
        
};

template<typename T> std::string Config::T_to_string(const T& t)
{
    std::ostringstream ost;
    ost << t;
    return ost.str();
}

template<> 
inline std::string Config::string_to_T<std::string>(const std::string& s)
{
    return s;
}

template<> 
inline bool Config::string_to_T<bool>(const std::string& s)
{
    bool b = true;
    std::string sup = s;
    for(std::string::iterator p = sup.begin(); p!= sup.end(); ++p)
        *p = toupper(*p);
    if(sup == std::string("FALSE")||sup == std::string("F")||
       sup == std::string("NO")||sup == std::string("N")||
       sup == std::string("0")||sup == std::string("NONE"))
       b = false;
    return b;
}

template<typename T> T Config::string_to_T(const std::string& s)
{
    T t;
    std::istringstream ist(s);
    ist >> t;
    return t;
}

template< typename T> T Config::get(const std::string& key) 
{
    map_const_iterator p = config_->config_map_.find(key);
    if(p == config_->config_map_.end()) throw KEY_NOT_FOUND(key);
    return string_to_T<T>(p->second);
}
template< typename T> T Config::get(const std::string& key, 
                                        const T& in_value)
{
    map_const_iterator p = config_->config_map_.find(key);
    if(p == config_->config_map_.end()) return in_value;
    return string_to_T<T>(p->second);
}

template< typename T> 
bool Config::getInfo(T& out_var, const std::string& key) const
{
    map_const_iterator p = config_map_.find(key);
    bool ifFound = (p != config_map_.end());
    if(p != config_map_.end()) 
        out_var = string_to_T<T>(p->second);
    return ifFound;
}

template< typename T>
bool Config::getInfo(T& out_var, const std::string& key, const T& in_value) const
{
    map_const_iterator p = config_map_.find(key);
    bool ifFound = (p != config_map_.end());
    if(p != config_map_.end()) 
        out_var = string_to_T<T>(p->second);
    else 
        out_var = in_value;
    return ifFound;
}

template< typename T> void Config::addKey(const std::string& key, const T& in_value)
{
    std::string var = T_to_string(in_value);
    std::string in_key = key;
    trim(in_key);
    trim(var);
    config_map_[in_key] = var;
    return;
}
}

#endif