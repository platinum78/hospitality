#ifndef ERRORS_HPP_
#define ERRORS_HPP_

#include <string>
#include <exception>

class ZeroDivError : public std::exception
{
public:
    ZeroDivError(const char *msg = "Division by zero is unacceptable.") : msg_(msg) {}
    // ~ZeroDivError();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class DimensionError : public std::exception
{
public:
    DimensionError(const char *msg = "Dimension mismatch.") : msg_(msg) {}
    // ~DimensionError();
    const char *what() { return msg_.c_str(); }

private:
    std::string msg_;
};


class ArgumentError : public std::exception
{
public:
    ArgumentError(const char *msg = "Invalid argument(s).") : msg_(msg) {}
    // ~ArgumentError();
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