#include "../include/utils/csv.hpp"

#include <list>
#include <string>

int main(void)
{
    CSVParser parser("../data/map/csv/places.csv");
    std::list<std::list<std::string> > dataContainer;
    parser.Parse(dataContainer);

    std::list<std::list<std::string> >::iterator rowIter = dataContainer.begin();
    std::list<std::string>::iterator colIter = rowIter->begin();

    for (; rowIter != dataContainer.end(); rowIter++)
    {
        for (colIter = rowIter->begin(); colIter != rowIter->end(); colIter++)
            printf("%30s \t", colIter->c_str());
        printf("\n");
    }
    printf("Here! \n");
}