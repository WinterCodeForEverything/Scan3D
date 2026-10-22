#ifndef SCAN3D_UTIL_LINE_STREAM_H
#define SCAN3D_UTIL_LINE_STREAM_H

#include <iostream>
#include <sstream>


namespace scan3d {

    namespace io {


        class LineInputStream {
        public:
            LineInputStream(std::istream &in) : in_(in), line_in_(nullptr) {}

            ~LineInputStream() {
                delete line_in_;
                line_in_ = nullptr;
            }

            bool eof() const { return in_.eof(); }

            bool eol() const { return line_in_ == nullptr || line_in_->eof(); }

            bool fail() const { return in_.fail() || line_in_->fail(); }

            void get_line() {
                getline(in_, buffer_);
                delete line_in_;
                line_in_ = new std::istringstream(buffer_);
            }

            std::istream &line() {
                assert(line_in_ != nullptr);
                return *line_in_;
            }

            const std::string &current_line() const {
                return buffer_;
            }

            template<class T>
            LineInputStream &operator>>(T &param) {
                *line_in_ >> param;
                return *this;
            }

        private:
            std::istream &in_;
            std::istringstream *line_in_;
            std::string buffer_;
        };


    } // namespace io

} // namespace scan3d


#endif  // SCAN3D_UTIL_LINE_STREAM_H
