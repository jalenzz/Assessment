#include <iostream>

class Matrix {
public:
    Matrix(int rows, int cols);
    Matrix(const Matrix& other);
    Matrix(Matrix&& other);
    Matrix(int rows, int cols, int** matrix);
    ~Matrix();

    Matrix& operator=(const Matrix& other);
    Matrix& operator=(Matrix &&other);
    Matrix operator+(const Matrix& other);
    Matrix operator-(const Matrix& other);
    Matrix operator*(Matrix &other);
    bool operator==(const Matrix& other);

    friend std::istream& operator>>(std::istream& in, Matrix& matrix);   
    friend std::ostream& operator<<(std::ostream& os, const Matrix& matrix);

private:
    int rows;
    int cols;
    int **matrix;
};

Matrix::Matrix(int rows, int cols): rows(rows), cols(cols) {
    this->matrix = new int*[rows];
    for (int i = 0; i < rows; i++) {
        this->matrix[i] = new int[cols];
    }
}
Matrix::Matrix(const Matrix& other) {
    this->rows = other.rows;
    this->cols = other.cols;
    this->matrix = new int*[rows];
    for (int i = 0; i < rows; i++) {
        this->matrix[i] = new int[cols];
    }
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            this->matrix[i][j] = other.matrix[i][j];
        }
    }
}
Matrix::Matrix(Matrix&& other) {
    this->rows = other.rows;
    this->cols = other.cols;
    this->matrix = new int*[rows];
    for (int i = 0; i < rows; i++) {
        this->matrix[i] = new int[cols];
    }
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            this->matrix[i][j] = other.matrix[i][j];
        }
    }
}
Matrix::Matrix(int rows, int cols, int **matrix) {
    this->rows = rows;
    this->cols = cols;
    this->matrix = matrix;
}
Matrix::~Matrix() {
    for (int i = 0; i < this->rows; i++) {
        delete[] this->matrix[i];
    }
    delete[] this->matrix;
}


Matrix& Matrix::operator=(const Matrix& other) {
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            this->matrix[i][j] = other.matrix[i][j];
        }
    }
    return *this;
}
Matrix &Matrix::operator=(Matrix &&other) {
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            this->matrix[i][j] = other.matrix[i][j];
        }
    }
    return *this;
}
Matrix Matrix::operator+(const Matrix& other) {
    Matrix result(this->rows, this->cols);
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            result.matrix[i][j] = this->matrix[i][j] + other.matrix[i][j];
        }
    }
    return result;
}
Matrix Matrix::operator-(const Matrix& other) {
    Matrix result(this->rows, this->cols);
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            result.matrix[i][j] = this->matrix[i][j] - other.matrix[i][j];
        }
    }
    return result;
}
Matrix Matrix::operator*(Matrix &other)
{
    Matrix result(this->rows, other.cols);
    for (int i = 0; i < result.rows; i++) {
        for (int j = 0; j < result.cols; j++) {
            result.matrix[i][j] = 0;
            for (int k = 0; k < this->cols; k++) {
                result.matrix[i][j] += this->matrix[i][k] * other.matrix[k][j];
            }
        }
    }
    return result;
}

bool Matrix::operator==(const Matrix& other) {
    bool flag = true;
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            if (this->matrix[i][j] != other.matrix[i][j]) {
                flag = false;
            }
        }
    }
    return flag;
}

std::istream& operator>>(std::istream& in, Matrix& matrix) {
    for (int i = 0; i < matrix.rows; i++) {
        for (int j = 0; j < matrix.cols; j++) {
            in >> matrix.matrix[i][j];
        }
    }
    return in;
}
std::ostream& operator<<(std::ostream& os, const Matrix& matrix) {
    for (int i = 0; i < matrix.rows; i++) {
        for (int j = 0; j < matrix.cols; j++) {
            os << matrix.matrix[i][j] << ' ';
        }
        os << '\n';
    }
    return os;
}
