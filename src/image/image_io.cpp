#include "image_io.hpp"

namespace pac
{

void SearchDir(std::string DirPath, std::vector<std::string> &_filePaths)
{
    std::vector<std::string> filePaths;
    int i, dirElements;
    std::string searchPath;

    struct stat statBuf;
    struct dirent **nameList = NULL;

    if (*DirPath.rbegin() != '/')
    {
        DirPath.push_back('/');
    }

    // dirElements にはディレクトリ内の要素数が入る
    dirElements = scandir(DirPath.c_str(), &nameList, NULL, NULL);

    if (dirElements == -1)
    {
        std::cout << "ERROR" << std::endl;
    }
    else
    {

        //ディレクトリかファイルかを順番に識別
        for (i = 0; i < dirElements; i += 1)
        {

            // "." と ".." を除く
            if ((strcmp(nameList[i]->d_name, ".\0") != 0) && (strcmp(nameList[i]->d_name, "..\0") != 0))
            {

                //search_pathには検索対象のフルパスを格納する
                searchPath = DirPath + std::string(nameList[i]->d_name);

                // ファイル情報の取得の成功
                if (stat(searchPath.c_str(), &statBuf) == 0)
                {

                    // ディレクトリだった場合の処理
                    if ((statBuf.st_mode & S_IFMT) == S_IFDIR)
                    {
                        // 再帰によりディレクトリ内を探索
                        SearchDir(DirPath + std::string(nameList[i]->d_name) + "/", filePaths);
                    }

                    //ファイルだった場合の処理
                    else
                    {
                        filePaths.push_back(searchPath);
                    }
                }

                // ファイル情報の取得の失敗
                else
                {
                    std::cout << "ERROR" << std::endl
                              << std::endl;
                }
            }
        }
    }
    std::sort(filePaths.begin(), filePaths.end());
    _filePaths = filePaths;
    free(nameList);
}

cv::Mat readColorImage(const std::string &filePath){
    cv::Mat image = cv::imread(filePath, 1);
    if (!image.data) {
        fprintf(stderr, "error: image does not exist\n");
        exit(1);
    }
    return image;
}

cv::Mat readGrayImage(const std::string &filePath){
    cv::Mat image = cv::imread(filePath, 0);
    if (!image.data) {
        fprintf(stderr, "error: image does not exist\n");
        exit(1);
    }
    return image;
}

void showImage(const cv::Mat &image){
    cvNamedWindow("Display Image", CV_WINDOW_AUTOSIZE);
    cv::imshow("Display Image", image);
    cvWaitKey(0);
}

} // namespace pac