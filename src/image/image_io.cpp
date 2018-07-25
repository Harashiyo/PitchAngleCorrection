//
// Created by Haraoka Shohei on 2018/07/26.
//

#include "image_io.h"

namespace pitchanglecorrection {
namespace image {

void SearchDir(std::string path, std::vector<std::string> &output) {
    int i, dirElements;
    std::string search_path;

    struct stat stat_buf;
    struct dirent **namelist = NULL;

    // dirElements にはディレクトリ内の要素数が入る
    dirElements = scandir(path.c_str(), &namelist, NULL, NULL);

    if (dirElements == -1) {
        std::cout << "ERROR" << std::endl;
    } else {

        //ディレクトリかファイルかを順番に識別
        for (i = 0; i < dirElements; i += 1) {

            // "." と ".." を除く
            if ((strcmp(namelist[i]->d_name, ".\0") != 0) && (strcmp(namelist[i]->d_name, "..\0") != 0)) {

                //search_pathには検索対象のフルパスを格納する
                search_path = path + std::string(namelist[i]->d_name);

                // ファイル情報の取得の成功
                if (stat(search_path.c_str(), &stat_buf) == 0) {

                    // ディレクトリだった場合の処理
                    if ((stat_buf.st_mode & S_IFMT) == S_IFDIR) {
                        // 再帰によりディレクトリ内を探索
                        SearchDir(path + std::string(namelist[i]->d_name) + "/", output);
                    }

                        //ファイルだった場合の処理
                    else {
                        output.push_back(search_path);
                    }
                }

                    // ファイル情報の取得の失敗
                else {
                    std::cout << "ERROR" << std::endl << std::endl;
                }
            }
        }
    }
    std::sort(output.begin(), output.end());

    free(namelist);
    return;
}

}
}