#pragma once
#include "nlohmann_json.hpp"
#include <iomanip>

#include <thread>
#include <mutex>


using Json = nlohmann::json;

#define LOCK_JSON() std::lock_guard<std::recursive_mutex> lock##__LINE__(JsonUtils::mtx());

class JsonUtils
{
public:
    static std::recursive_mutex& mtx()
    {
        return m_mtx;
    }

    static bool write_json_to_file(const std::string& file, const Json& j)
    {
        Json j_base;
        if (read_json_from_file(file, false, &j_base))
        {
            if (j_base == j)
            {
                // printf("json is same with file\n");
                return true;
            }
        }
        if (j.is_null())
        {
            return true;
        }
        bool success = true;
        std::ofstream o(file);
        if (o)
        {
            o << std::setw(4) << j << std::endl;
            //o << j << std::endl;
            o.close();
        }
        else
        {
            success = false;
            ////PRINT_ERROR("can't write to file %s", file.c_str());
        }
        return success;
    }

    static Json read_json_from_file(const std::string& file)
    {
        Json j;
        try
        {
            std::ifstream i(file);
            if (i)
            {
                i >> j;
            }
            else
            {
                //PRINT_ERROR("can't open %s, create a new one", file.c_str());
                write_json_to_file(file, j);
            }
        }
        catch (Json::parse_error& ex)
        {
            printf("parse exception: %s\n", ex.what());
            //PRINT_ERROR("parse exception: %s", ex.what());
        }
        catch (std::exception& e)
        {
            j.clear();
            //PRINT_ERROR("exception: %s", e.what());
        }
        catch (...)
        {
            j.clear();
            //PRINT_ERROR("unknown error");
        }

        return j;
    }

    static bool read_json_from_file(const std::string& file, Json* p_j)
    {
        return read_json_from_file(file, true, p_j);
    }

    static bool read_json_from_file(const std::string& file, bool create_new_one, Json* p_j)
    {
        bool result = false;
        if (p_j != nullptr)
        {
            try
            {
                std::ifstream i(file);
                if (i)
                {
                    i >> (*p_j);
                    result = true;
                    i.close();
                }
                else
                {
                    i.close();
                    if (create_new_one)
                    {
                        //PRINT_ERROR("can't open %s, create a new one", file.c_str());
                        Json j;
                        write_json_to_file(file, j);
                        *p_j = j;
                        result = true;
                    }
                }
            }
            catch (Json::parse_error& ex)
            {
                printf("parse exception: %s\n", ex.what());
                //PRINT_ERROR("parse exception: %s", ex.what());
                if (create_new_one)
                {
                    *p_j = Json::object();
                }
                result = false;
            }
            catch (std::exception& e)
            {
                //PRINT_ERROR("exception: %s", e.what());
                if (create_new_one)
                {
                    *p_j = Json::object();
                }
                //
                result = create_new_one;
            }
            catch (...)
            {
                //PRINT_ERROR("unknown error");
                if (create_new_one)
                {
                    *p_j = Json::object();
                }
                result = create_new_one;
            }
        }
        return result;
    }

    static bool read_json_from_str(const std::string& str, Json* p_j)
    {
        bool result = false;
        if (p_j != nullptr)
        {
            try
            {
                *p_j = Json::parse(str);
                result = true;
            }
            catch (Json::parse_error& ex)
            {
                printf("parse exception: %s\n", ex.what());
                //PRINT_ERROR("parse exception: %s", ex.what());
            }
            catch (std::exception& e)
            {
                //PRINT_ERROR("exception: %s", e.what());
                *p_j = {};
                result = false;
            }
            catch (...)
            {
                //PRINT_ERROR("unknown error");
                *p_j = {};
                result = false;
            }
        }
        return result;
    }

    static Json read_json_from_str(const std::string& str)
    {
        Json j;

        try
        {
            j = Json::parse(str);
        }
        catch (Json::parse_error& ex)
        {
            printf("parse exception: %s\n", ex.what());
            //PRINT_ERROR("parse exception: %s", ex.what());
        }
        catch (std::exception& e)
        {
            //PRINT_ERROR("exception: %s", e.what());
            j = {};
        }
        catch (...)
        {
            //PRINT_ERROR("unknown error");
            j = {};
        }

        return j;
    }

    static bool json_contains(const Json& j, const std::string& key)
    {
        return (j.contains(key) && !j[key].is_null());
    }

    template <class T>
    static bool get_json_data(Json* p_j, const std::string& key, T* p_value,
                              const bool print_warn = true)
    {
        if (p_j == nullptr)
        {
            return false;
        }
        Json& j = (*p_j);

        if (j.find(key) != j.end() && p_value != nullptr)
        {
            // T value = static_cast<T>(j[key]);
            T value = j[key];
            *p_value = std::move(value);
            //*p_value = static_cast<T>(j[key]);
            //*p_value = (T)(j[key]);
            return true;
        }
        else
        {
            if (print_warn)
            {
                if (j.contains(key))
                {
                    //PRINT_ERROR("j[%s] is null: %d", key.c_str(), j[key].is_null());
                }
                else
                {
                    //PRINT_ERROR("can't find %s in json", key.c_str());
                }
                if (p_value == nullptr)
                {
                    //PRINT_ERROR("p_value = null");
                }
            }
        }
        return false;
    }

    template <class T>
    static bool get_json_data(const Json& j, const std::string& key, T* p_value,
                              const bool print_warn = true)
    {
        if (j.contains(key) && !j[key].is_null() && p_value != nullptr)
        {
            T value = (j[key]);
            *p_value = value;
            //*p_value = (T)(j[key]);
            return true;
        }
        else
        {
            if (print_warn)
            {
                if (j.contains(key))
                {
                    //PRINT_ERROR("j[%s] is null: %d", key.c_str(), j[key].is_null());
                }
                else
                {
                    //PRINT_ERROR("can't find %s in json", key.c_str());
                }
                if (p_value == nullptr)
                {
                    //PRINT_ERROR("p_value = null");
                }
            }
        }
        return false;
    }

    template <class T>
    static bool get_json_data(const Json& j, const std::string& key, const T& default_value,
                              T* p_value, const bool print_warn = true)
    {
        if (get_json_data(j, key, p_value, print_warn))
        {
            //*p_value = static_cast<T>(j[key]);
            //*p_value = (T)(j[key]);
            return true;
        }
        if (p_value != nullptr)
        {
            *p_value = default_value;
        }
        return false;
    }

    template <class T>
    static bool get_json_data(Json& j, const std::string& key, const T& default_value, T* p_value,
                              const bool print_warn = true)
    {
        if (get_json_data(&j, key, p_value, print_warn))
        {
            return true;
        }
        else
        {
            j[key] = default_value;
        }
        if (p_value != nullptr)
        {
            *p_value = default_value;
        }

        return false;
    }

    template <class T>
    static bool get_json_data(Json* p_j, const std::string& key, const T& default_value, T* p_value,
                              const bool print_warn = true)
    {
        if (p_j == nullptr)
        {
            return false;
        }

        if (get_json_data(p_j, key, p_value, print_warn))
        {
            return true;
        }
        else
        {
            (*p_j)[key] = default_value;
        }
        if (p_value != nullptr)
        {
            *p_value = default_value;
        }

        return false;
    }

    template <class T>
    static T get_json_data(const Json& j, const std::string& key, const T& default_value,
                           const bool print_warn = true)
    {
        T value;
        if (get_json_data(j, key, &value, print_warn))
        {
            //*p_value = static_cast<T>(j[key]);
            //*p_value = (T)(j[key]);
            return value;
        }
        return default_value;
    }

    template <class T>
    static T get_json_data(Json& j, const std::string& key, const T& default_value,
                           const bool print_warn = true)
    {
        T value;
        if (get_json_data(&j, key, &value, print_warn))
        {
            return value;
        }
        else
        {
            j[key] = default_value;
        }
        return default_value;
    }

    template <class T>
    static T get_json_data(Json* p_j, const std::string& key, const T& default_value,
                           const bool print_warn = false)
    {
        T value;
        if (p_j == nullptr)
        {
            //PRINT_WARN("pointer j is null, return default value");
            return default_value;
        }

        if (get_json_data(p_j, key, &value, print_warn))
        {
            return value;
        }
        else
        {
            (*p_j)[key] = default_value;
        }
        return default_value;
    }

private:
    static std::recursive_mutex m_mtx;
};

