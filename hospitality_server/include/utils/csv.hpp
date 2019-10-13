#include <cstdio>
#include <cstring>
#include <vector>
#include <list>
#include <string>

class CSVParser
{
public:
    CSVParser(std::string csv_path);
    ~CSVParser();

public:
    void Parse(std::list<std::list<std::string> > &data_container);

private:
    std::string csv_path_;
    char *csv_data_buffer_;
};

CSVParser::CSVParser(std::string csv_path)
{
    csv_path_.assign(csv_path);
    FILE *fp = fopen(csv_path_.c_str(), "r");
    fseek(fp, 0, SEEK_END);
    int csvLen = ftell(fp);
    csv_data_buffer_ = new char[csvLen + 1];
    fseek(fp, 0, SEEK_SET);
    fread(csv_data_buffer_, 1, csvLen, fp);
    csv_data_buffer_[csvLen] = '\0';
}

void CSVParser::Parse(std::list<std::list<std::string> > &data_container)
{
    int begin = 0, end = 0, cursor = 0;
    enum State { DATA, PENDING, IDLE };
    State dataState = IDLE;
    int csvLen = strlen(csv_data_buffer_);
    data_container.resize(0);
    data_container.push_back(std::list<std::string>());
    std::list<std::list<std::string> >::iterator iter = data_container.begin();
    
    while (cursor < csvLen)
    {
        if (csv_data_buffer_[cursor] == ',')
        {
            // 데이터 스캔 중 또는 스캔 후 콤마를 만났을 경우
            if (dataState != IDLE)
            {
                if (dataState == DATA)
                    end = cursor;
                csv_data_buffer_[end] = '\0';
                std::string str(csv_data_buffer_ + begin);
                csv_data_buffer_[end] = ',';
                iter->push_back(str);
                dataState = IDLE;
            }
            // 데이터가 없는 상태에서 콤마를 만났을 경우
            else
            {
                end = cursor;
                std::string str;
                iter->push_back(str);
            }   
        }
        else if (csv_data_buffer_[cursor] == ' ')
        {
            // 데이터 스캔 중 공백을 만났을 경우
            if (dataState == DATA)
            {
                end = cursor;
                dataState = PENDING;
            }
        }
        else if (csv_data_buffer_[cursor] == '\n')
        {
            if (dataState == DATA || dataState == PENDING)
            {
                if (dataState == DATA)
                    end = cursor;
                csv_data_buffer_[end] = '\0';
                std::string str(csv_data_buffer_ + begin);
                iter->push_back(str);
            }
            else
            {
                iter->push_back(std::string());
            }
            data_container.push_back(std::list<std::string>());
            iter++;
            dataState = IDLE;
        }
        else
        {
            if (dataState == IDLE)
            {
                begin = end = cursor;
                dataState = DATA;
            }
            else if (dataState == PENDING)
            {
                end = cursor;
                dataState = DATA;
            }
            else if (dataState = DATA)
                end = cursor;
        }
        
        cursor++;
    }

    if (iter->size() == 0)
        data_container.erase(iter);
}

CSVParser::~CSVParser()
{
    delete[] csv_data_buffer_;
}