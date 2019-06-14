#include "pcl_viewer_pose/config.h"


namespace pcl_viewer_pose
{    
    void Config::setConfigFile(const std::string& filename,
                               std::string delimiter,
                               std::string comment)
    {
        if (config_ == nullptr)
            config_ = boost::shared_ptr<Config>(new Config);
        config_->config_file_.open(filename);
        if (!config_->config_file_) throw FILE_NOT_FOUND(filename);
        std::string line;
        if (config_->config_file_.is_open())
        {
            while (getline(config_->config_file_, line))
            {
                // ignore comments
                line = line.substr(0, line.find(comment));
                
                std::size_t deliIdx = line.find(delimiter);
                if(deliIdx != std::string::npos)
                {
                    std::string key = line.substr(0, deliIdx);
                    line.replace(0, deliIdx + delimiter.length(), "");
                    Config::trim(key);
                    Config::trim(line);
                    config_->config_map_[key] = line;
                }
            }
        }
        else
        {
            std::cerr << "Config File " << filename << "does not exist." 
                      << std::endl;
            config_->config_file_.close();
            return;
        }
    }

    bool Config::ifFileExist(std::string filename)
    {
        std::ifstream file(filename);
        if(file.is_open())
        {
            return true;
        }
        else
        {
            return false;
        }
        
    }

    bool Config::ifKeyExist(const std::string& key) const
    {
        map_const_iterator p = config_map_.find(key);
        return(p != config_map_.end());
    }

    void Config::remove(const std::string& key)
    {
        config_map_.erase(config_map_.find(key));
    }

    void Config::trim(std::string& s)
    {
        static const char whitespace[] = "\n\t\v\r\f";
        s.erase(0, s.find_first_not_of(whitespace));
        s.erase(s.find_last_not_of(whitespace) + 1U);
    }

    Config::~Config()
    {
        if (config_file_.is_open())
            config_file_.close();
    }

    boost::shared_ptr<Config> Config::config_ = nullptr;
}