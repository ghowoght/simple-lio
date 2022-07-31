/**
 * @file file_helper.hpp
 * @author Linfu Wei (ghowoght@qq.com)
 * @brief 
 * @version 1.0
 * @date 2022-07-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef FILE_HELPER_HPP
#define FILE_HELPER_HPP

#include <fstream>
#include <memory>

////// FileWriter //////
class FileWriter{
private:
    std::ofstream file_;
public:
    FileWriter(const std::string& file_name){
        file_.open(file_name, std::ios::out);
    }
    ~FileWriter(){
        file_.close();
    }
    void write_txt(const std::string& str){
        file_ << str;
    }
    void write_bin(const char* bytes, size_t size){
        file_.write(bytes, size);
    }
    void close(){
        file_.close();
    }
    static std::shared_ptr<FileWriter> create(const std::string& file_name){
        return std::make_shared<FileWriter>(file_name);
    }
};
using FileWriterPtr = std::shared_ptr<FileWriter>;

////// FileReader //////
class FileReader{
private:
    std::ifstream file_;
public:
    enum{
        TXT = 0,
        BIN,
    };
public:
    FileReader(const std::string& file_name, int type){
        if(type == BIN)
            file_.open(file_name, std::ios::binary);
        else
            file_.open(file_name, std::ios::in);
    }
    ~FileReader(){
        file_.close();
    }
    void read_txt(double* buf, size_t size){
        for(size_t i = 0; i < size; ++i){
            file_ >> buf[i];
        }
    }
    void read_bin(char* bytes, size_t size){
        file_.read(bytes, size);
    }
    void close(){
        file_.close();
    }
    static std::shared_ptr<FileReader> create(const std::string& file_name, int type = TXT){
        return std::make_shared<FileReader>(file_name, type);
    }
};
using FileReaderPtr = std::shared_ptr<FileReader>;

#endif // FILE_HELPER_HPP