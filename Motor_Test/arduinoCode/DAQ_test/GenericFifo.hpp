#pragma once

#include <iostream>
#include <array>
#include <stdexcept>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

namespace fifolib::generic {

    template<std::size_t max_message_size>
    class GenericFifoReader;

    template<std::size_t max_message_size>
    class GenericFifoWriter;

    template<std::size_t max_message_size>
    GenericFifoReader<max_message_size>* open_reader(const GenericFifoWriter<max_message_size>& writer, const std::size_t timeout_tries);
    /**
     * GenericFifoBody is a class that holds the shared objects for the generic FIFO. This class should not be called by the user.
     * @tparam max_message_size Size of the largest message that will go into the FIFO in bytes.
     */
    template<std::size_t max_message_size>
    class GenericFifoBody {
    public:

        /**
         * A type large enough to store any message. The buffer itself will be an array of cell_types.
         */
        using cell_type = std::array<uint8_t, max_message_size>;

        /*
         * Boolean flag which keeps track of whether or not there is currently an active writer for the FIFO. This is to ensure that calls to resume_writer are unsuccessful when a writer already exists. 
         */
        volatile bool has_active_writer;

        /*
         * Boolean flag which keeps track of whether or not there is currently an active reader for the FIFO. This is to ensure that calls to open_reader are unsuccessful when a reader already exists. 
         */
        volatile bool has_active_reader;

        /**
         * The unsigned variable which stores the number of messages that have been successfully written. This is used to determine the position of the writer inside the buffer.
         * @note It should be noted that write_index never resets itself. As a result, the number of messages that can be written to the FIFO is "only" 2^64. Although I sincerely doubt that this number will ever be reached (given the throughput of the FIFO, it would take 31,700 years of continuous reading and writing for integer overflow to occur), in the event that you want this FIFO to continue working for a dozen orders of magnitude more than the lifespan of the universe, "std::size_t" can always be replaced by "unsigned __int128".
         */
        volatile std::size_t write_index;

        /**
         * The unsigned variable which stores the number of messages that have been successfully read. This is used to determine the position of the reader inside the buffer. As with the writer, this variable will break after 2^64 messages have been read. If need be, "std::size_t" can be replaced by "unsigned __int128".
         */
        volatile std::size_t read_index;

        /**
         * The number of bytes that come before the beginning of the array of messages inside the GenericFifoBody.
         */
        static constexpr const std::size_t buffer_ofs = sizeof(GenericFifoBody<max_message_size>);

        /**
         * Returns the pointer to the beginning of the array of messages, made up of cell_types.
         * @return Pointer to the beginning of the message buffer.
         */
//      public:
        cell_type* buffer() {
            return reinterpret_cast<cell_type*> (reinterpret_cast<uint8_t*>(this) + buffer_ofs);
        }

        GenericFifoBody() = delete;
    };

    /**
     * The generic FIFO imposes no restriction on the type of message that can be passed. This, however, comes at a performance penalty. It is a lossless, single-producer, single-consumer type FIFO which uses a ring buffer design to buffer messages.
     *
     * GenericFifoReader is the class which provides reader functionality for the generic FIFO.
     * @tparam max_message_size Size of the largest message that will go into the FIFO in bytes.
     */
    template<std::size_t max_message_size>
    class GenericFifoReader {
    protected:

        /**
         * A pointer to the body of the FIFO.
         */
        GenericFifoBody<max_message_size>* body;

        /**
         * The length of the FIFO, as a number of messages.
         */
        const std::size_t fifo_size;

    public:

        /**
         * Constructor of the GenericFifoReader. This should not be called by the user.
         */
        GenericFifoReader(GenericFifoBody<max_message_size>* body, const std::size_t fifo_size) : body(body), fifo_size(fifo_size) {};

        // Due to the mmapped segment, you can move, but you cannot copy
        GenericFifoReader(const GenericFifoReader&) = delete;
        GenericFifoReader(GenericFifoReader&& that) : body(that.body) {
            that.body = nullptr;
        }
    
        GenericFifoReader& operator=(const GenericFifoReader&) = delete;
        GenericFifoReader& operator=(GenericFifoReader&& that) {
            body = that.body;
            that.body = nullptr;
        }
    public:

        /**
         * Computes the difference between the write_index and the read_index.
         * @return The number of messages between the writer and reader.
         */
        std::size_t num_messages_to_read() const {
            const std::size_t result = body->write_index - body->read_index;
            return result;
        }

        /**
         * If the difference between write_index and read_index is not 0, then the read can happen.
         * @return Whether or not a read can happen.
         */
        bool can_read() const {
            return num_messages_to_read() != 0;
        }

    public:

        /**
         * Evaluates whether or not the message buffer is full. This is the opposite of can_read.
         * @return Whether or not the buffer is empty.
         */
        bool empty() const {
            return !can_read();
        }

        /**
         * Method used to peek at the next message and figure out what its actual type is. This is for when you are unsure of the type of the next message. Returns nullptr if a read can not happen.
         * @tparam T Guess type, use its fields to figure out the actual type.
         * @return Pointer to next message.
         */
        template<typename T>
        const T* try_peek() {
            if (!can_read()) {
                return nullptr;
            }

            return reinterpret_cast<const T*>(&body->buffer()[body->read_index%fifo_size]);
        }

        /**
         * This is the method to read messages once you know the type. The method will memcpy from the buffer and into the reference provided as a parameter. The method returns true upon success, false upon failure. This particular overload of try_read does not require all messages to be aligned.
         * @tparam T Type of message.
         * @param msg Reference to aligned message.
         * @return Whether or not the read was successful.
         */
        template<typename T>
        bool try_read(T& msg) {

            if (!can_read()) {
                return false;
            }

            memcpy(reinterpret_cast<void*>(&msg), &body->buffer()[body->read_index%fifo_size], sizeof(T));

            body->read_index = body->read_index + 1;
            return true;
        }

        void clear_buffer() {
          body->read_index = body->write_index;
        }

        /**
         * If the next message does not need to be copied and can be ignored, then this message simply skips it.
         * @return Whether or not the skip was successful.
         */
        bool try_skip() {
            if (can_read()) {
                body->read_index = body->read_index + 1;
                return true;
            }
            return false;
        }

        ~GenericFifoReader() {
            body->has_active_reader = false;
        }
    };

    /**
     * The generic FIFO imposes no restriction on the type of message that can be passed. This, however, comes at a performance penalty. It is a lossless, single-producer, single-consumer type FIFO which uses a ring buffer design to buffer messages.
     *
     * GenericFifoWriter is the class which provides writer functionality for the generic FIFO.
     * @tparam max_message_size Size of the largest message that will go into the FIFO in bytes.
     */
    template<std::size_t max_message_size>
    class GenericFifoWriter {
    protected:

        /**
         * A pointer to the body of the FIFO.
         */
        GenericFifoBody<max_message_size>* body;

        /**
         * The length of the FIFO, as a number of messages.
         */
        const std::size_t fifo_size;

        std::size_t failed_writes;

    public:

        /**
         * Constructor of the GenericFifoWriter. This should not be called by the user.
         */
        GenericFifoWriter(GenericFifoBody<max_message_size>* body, const std::size_t fifo_size) : body(body), fifo_size(fifo_size), failed_writes(0) {};

        // Due to the mmapped segment, you can move, but you cannot copy
        GenericFifoWriter(const GenericFifoWriter&) = delete;
        GenericFifoWriter(GenericFifoWriter&& that) : body(that.body), failed_writes(that.getFailedWrites()) {
            that.body = nullptr;
        }
    
        GenericFifoWriter& operator=(const GenericFifoWriter&) = delete;
        GenericFifoWriter& operator=(GenericFifoWriter&& that) {
            body = that.body;
            that.body = nullptr;
            failed_writes = that.getFailedWrites();
        }

        template<std::size_t max_message_size_>
        friend GenericFifoReader<max_message_size_>* open_reader(GenericFifoWriter<max_message_size_>& writer, const std::size_t timeout_tries);

    protected:
        /**
         * Computes the difference between the write_index and the read_index.
         * @return The number of messages between the writer and reader.
         */
        std::size_t num_messages_to_read() const {
            const std::size_t result = body->write_index - body->read_index;
//            assert(result <= fifo_size);
            return result;
        }

        /**
         * If the difference between write_index and read_index is less than the length of the FIFO, then the write can happen.
         * @return Whether or not a write can happen.
         */
        bool can_write() const {
            return num_messages_to_read() < fifo_size;
        }

    public:

        /**
         * This is the method to write messages. The method will memcpy from the message reference and into the buffer. The method returns true upon success, false upon failure. This overload of try_write does not require all messages to be aligned.
         * @tparam T Type of message.
         * @param msg Reference to aligned message.
         * @return Whether or not the write was successful.
         */
        template<typename T>
        bool try_write(const T& msg) {
            static_assert(sizeof(T) <= max_message_size);
            if (!can_write()) {
                ++failed_writes;
                return false;
            }

            memcpy(&body->buffer()[body->write_index%fifo_size], &msg, sizeof(T));

            body->write_index = body->write_index + 1;
            return true;
        }

        std::size_t getFailedWrites() {
            return failed_writes;
        }

        ~GenericFifoWriter() {
            body->has_active_writer = false;
            free(body);
        }

        friend GenericFifoReader<max_message_size>* open_reader<max_message_size>(const GenericFifoWriter<max_message_size>& writer, const std::size_t timeout_tries);
    };

    /**
     * Method used to instantiate a generic FIFO. This overload creates a FIFO using aligned_alloc for inter-threaded communication. Returns a pointer to a GenericFifoWriter object.
     * @relates GenericFifoWriter
     * @tparam max_message_size Size of the largest message that will go into the FIFO in bytes.
     * @param fifo_size Size of the FIFO in terms of the number of messages it will hold.
     * @return A pointer to a GenericFifoWriter object.
     */
    template<std::size_t max_message_size>
    GenericFifoWriter<max_message_size>* init_writer(const std::size_t fifo_size) {
        // static_assert(sizeof(GenericFifoBody<max_message_size>) == sizeof(std::uint64_t) + sizeof(std::size_t)*2);
        const int len = sizeof(GenericFifoBody<max_message_size>) + fifo_size*sizeof(typename GenericFifoBody<max_message_size>::cell_type);
        auto* const ptr = reinterpret_cast<GenericFifoBody<max_message_size>*>(malloc(len));
        ptr->has_active_writer = true;
        ptr->has_active_reader = false;
        ptr->write_index = 0;
        ptr->read_index = 0;
        memset(ptr->buffer(), 0, fifo_size*sizeof(typename GenericFifoBody<max_message_size>::cell_type));
        return new GenericFifoWriter<max_message_size>(ptr, fifo_size);
    }

    /**
     * Method to open a generic FIFO reader. This overload creates one from a reference to a writer.
     * @relates GenericFifoReader
     * @tparam max_message_size Size of the largest message that will go into the FIFO in bytes.
     * @param writer Reference to writer object.
     * @param timeout_tries The number of times this method will check to see if there already exists an active reader for the given FIFO. This method will sleep for a millisecond after every attempt. If, at any point, it sees that there is no active reader, then it will step checking and proceed.
     * @return A pointer to a GenericFifoReader object.
     * @throws std::runtime_error If there is already an active reader, then the method fails.
     */
    template<std::size_t max_message_size>
    GenericFifoReader<max_message_size>* open_reader(const GenericFifoWriter<max_message_size>& writer, const std::size_t timeout_tries) {
        for (std::size_t tries = 0; tries < timeout_tries; ++tries) {
            if (!writer.body->has_active_reader) {
                break;
            }
            usleep(10000);
        }
        if (writer.body->has_active_reader) {
            std::cout << "fifolib: Multithread FIFO already has active reader." << std::endl;
//            throw 0;
            return nullptr;
        }
        writer.body->has_active_reader = true;
        return new GenericFifoReader<max_message_size>(writer.body, writer.fifo_size);
    }
}
