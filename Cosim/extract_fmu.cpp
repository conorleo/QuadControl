#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <windows.h>

// Error handling for ZIP operations
#define UNZIP_BUFFER_SIZE 8192

// Simple ZIP extraction using Windows API
// For production use, consider using minizip or ZipLib for cross-platform support

bool CreateDirectoryRecursive(const std::filesystem::path& path) {
    if (path.empty() || std::filesystem::exists(path)) {
        return true;
    }
    
    if (!CreateDirectoryRecursive(path.parent_path())) {
        return false;
    }
    
    return std::filesystem::create_directory(path);
}

// Extract ZIP file using SharpZipLib wrapper or command line
bool ExtractZipFile(const std::string& zipFilePath, const std::string& extractPath) {
    try {
        // Create extract directory if it doesn't exist
        std::filesystem::path extractDir(extractPath);
        if (!std::filesystem::exists(extractDir)) {
            std::filesystem::create_directories(extractDir);
            std::cout << "Created directory: " << extractPath << std::endl;
        }

        // Use PowerShell to extract ZIP (available on all modern Windows)
        std::string command = "powershell -Command \"Expand-Archive -Path '" + zipFilePath + 
                             "' -DestinationPath '" + extractPath + "' -Force\"";
        
        int result = system(command.c_str());
        
        if (result == 0) {
            std::cout << "Successfully extracted " << zipFilePath << " to " << extractPath << std::endl;
            return true;
        } else {
            std::cerr << "Failed to extract ZIP file. Command returned: " << result << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception during extraction: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char* argv[]) {
    // Default paths
    std::string zipFile = "Model/Quadcopter_fmu.zip";
    std::string extractPath = "Model";

    // Allow command-line override
    if (argc > 1) {
        zipFile = argv[1];
    }
    if (argc > 2) {
        extractPath = argv[2];
    }

    std::cout << "FMU Extraction Tool" << std::endl;
    std::cout << "==================" << std::endl;
    std::cout << "ZIP File: " << zipFile << std::endl;
    std::cout << "Extract To: " << extractPath << std::endl;
    std::cout << std::endl;

    // Check if ZIP file exists
    if (!std::filesystem::exists(zipFile)) {
        std::cerr << "Error: ZIP file not found: " << zipFile << std::endl;
        return 1;
    }

    // Extract the ZIP file
    if (ExtractZipFile(zipFile, extractPath)) {
        std::cout << "Extraction completed successfully!" << std::endl;
        
        // List extracted files
        std::cout << "\nExtracted files:" << std::endl;
        for (const auto& entry : std::filesystem::recursive_directory_iterator(extractPath)) {
            std::cout << "  " << entry.path().string() << std::endl;
        }
        
        return 0;
    } else {
        std::cerr << "Extraction failed!" << std::endl;
        return 1;
    }
}
