#ifndef ERRORS_HPP_
#define ERRORS_HPP_

#include <string>
#include <exception>

class ZeroDivException : public std::exception
{
public:
    ZeroDivException(const char *msg = "Division by zero is unacceptable.") : msg_(msg) {}
    // ~ZeroDivException();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class OutOfBoundException : public std::exception
{
public:
    OutOfBoundException(const char *msg = "Out of bound.") : msg_(msg) {}
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class DimensionException : public std::exception
{
public:
    DimensionException(const char *msg = "Dimension mismatch.") : msg_(msg) {}
    // ~DimensionException();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class ArgumentException : public std::exception
{
public:
    ArgumentException(const char *msg = "Invalid argument(s).") : msg_(msg) {}
    // ~ArgumentException();
    const char *what() const throw() { return msg_.c_str(); }

private:
    std::string msg_;
};


class NoSuchNodeException : public std::exception
{
public:
    NoSuchNodeException(const char *msg = "No such node.") : msg_(msg) {}
    // ~NoSuchNodeException();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class DuplicateNodeException : public std::exception
{
public:
    DuplicateNodeException(const char *msg = "Node already exists.") : msg_(msg) {}
    // ~DuplicateNodeException();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class NoSuchConnectionException : public std::exception
{
public:
    NoSuchConnectionException(const char *msg = "No such connection.") : msg_(msg) {}
    // ~NoSuchConnectionException();
    const char *what() { return msg_.c_str(); };

private:
    std::string msg_;
};


class DuplicateConnectionException : public std::exception
{
public:
    DuplicateConnectionException(const char *msg = "Connection already exists.") : msg_(msg) {}
    // ~DuplicateConnectionException();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class NoPathException : public std::exception
{
public:
    NoPathException(const char *msg = "Path does not exist.") : msg_(msg) {}
    // ~NoPathException();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};

class RuntimeException : public std::exception
{
public:
    RuntimeException(const char *msg = "Runtime exception.") : msg_(msg) {}
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};

#endif