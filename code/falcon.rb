require_relative 'command/serve'
require_relative 'command/virtual'

require_relative 'version'

require 'samovar'

module Falcon
	module Command
		def self.parse(*args)
			Top.parse(*args)
		end
		
		class Top < Samovar::Command
			self.description = "An asynchronous HTTP client/server toolset."
			
			options do
				option '--verbose | --quiet', "Verbosity of output for debugging.", key: :logging
				option '-h/--help', "Print out help information."
				option '-v/--version', "Print out the application version."
			end
			
			nested '<command>', {
				'serve' => Serve,
				'virtual' => Virtual
			}, default: 'serve'
			
			def verbose?
				@options[:logging] == :verbose
			end
			
			def quiet?
				@options[:logging] == :quiet
			end
			
			def invoke(program_name: File.basename($0))
				if verbose?
					Async.logger.level = Logger::DEBUG
				elsif quiet?
					Async.logger.level = Logger::WARN
				else
					Async.logger.level = Logger::INFO
				end
				
				if @options[:version]
					puts "falcon v#{Falcon::VERSION}"
				elsif @options[:help] or @command.nil?
					print_usage(program_name)
				else
					@command.invoke(self)
				end
			end
		end
	end
end


# Copyright, 2018, by Samuel G. D. Williams. <http://www.codeotaku.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

require 'async/http/url_endpoint'
require 'localhost/authority'

module Falcon
	class Endpoint < Async::HTTP::URLEndpoint
		def ssl_context
			@options[:ssl_context] || build_ssl_context
		end
		
		def build_ssl_context(hostname = self.hostname)
			authority = Localhost::Authority.fetch(hostname)
			
			authority.server_context.tap do |context|
				context.alpn_select_cb = lambda do |protocols|
					if protocols.include? "h2"
						return "h2"
					elsif protocols.include? "http/1.1"
						return "http/1.1"
					elsif protocols.include? "http/1.0"
						return "http/1.0"
					else
						return nil
					end
				end
				
				context.session_id_context = "falcon"
			end
		end
	end
end

require 'async/io/endpoint'

require_relative 'proxy'
require_relative 'redirection'

require 'async/container/forked'

module Falcon
	class Host
		def initialize
			@app = nil
			@app_root = nil
			@config_path = "config.ru"
			
			@endpoint = nil
			
			@ssl_certificate = nil
			@ssl_key = nil
			
			@ssl_context = nil
		end
		
		attr_accessor :app
		attr_accessor :app_root
		attr_accessor :config_path
		
		attr_accessor :endpoint
		
		attr_accessor :ssl_certificate
		attr_accessor :ssl_key
		
		attr_accessor :ssl_context
		
		def freeze
			return if frozen?
			
			ssl_context
			
			super
		end
		
		def app?
			@app || @config_path
		end
		
		def load_app(verbose = false)
			return @app if @app
			
			if @config_path
				rack_app, options = Rack::Builder.parse_file(@config_path)
				
				return Server.middleware(rack_app, verbose: verbose)
			end
		end
		
		def self_signed!(hostname)
			authority = Localhost::Authority.fetch(hostname)
			
			@ssl_context = authority.server_context.tap do |context|
				context.alpn_select_cb = lambda do |protocols|
					if protocols.include? "h2"
						return "h2"
					elsif protocols.include? "http/1.1"
						return "http/1.1"
					elsif protocols.include? "http/1.0"
						return "http/1.0"
					else
						return nil
					end
				end
				
				context.session_id_context = "falcon"
			end
		end
		
		def ssl_certificate_path= path
			@ssl_certificate = OpenSSL::X509::Certificate.new(File.read(path))
		end
		
		def ssl_key_path= path
			@ssl_key = OpenSSL::PKey::RSA.new(File.read(path))
		end
		
		def ssl_context
			@ssl_context ||= OpenSSL::SSL::SSLContext.new.tap do |context|
				context.cert = @ssl_certificate
				context.key = @ssl_key
				
				context.session_id_context = "falcon"
				
				context.set_params
				
				context.setup
			end
		end
		
		def start(*args)
			if self.app?
				Async::Container::Forked.new do
					Dir.chdir(@app_root) if @app_root
					
					app = self.load_app(*args)
					
					server = Falcon::Server.new(app, self.server_endpoint)
					
					server.run
				end
			end
		end
	end
	
	class Hosts
		DEFAULT_ALPN_PROTOCOLS = ['h2', 'http/1.1'].freeze
		
		def initialize
			@named = {}
			@server_context = nil
			@server_endpoint = nil
		end
		
		def each(&block)
			@named.each(&block)
		end
		
		def endpoint
			@server_endpoint ||= Async::HTTP::URLEndpoint.parse(
				'https://[::]',
				ssl_context: self.ssl_context,
				reuse_address: true
			)
		end
		
		def ssl_context
			@server_context ||= OpenSSL::SSL::SSLContext.new.tap do |context|
				context.servername_cb = Proc.new do |socket, hostname|
					self.host_context(socket, hostname)
				end
				
				context.session_id_context = "falcon"
				
				context.alpn_protocols = DEFAULT_ALPN_PROTOCOLS
				
				context.set_params
				
				context.setup
			end
		end
		
		def host_context(socket, hostname)
			if host = @named[hostname]
				socket.hostname = hostname
				
				return host.ssl_context
			end
		end
		
		def add(name, host = Host.new, &block)
			host = Host.new
			
			yield host if block_given?
			
			@named[name] = host.freeze
		end
		
		def client_endpoints
			Hash[
				@named.collect{|name, host| [name, host.endpoint]}
			]
		end
		
		def proxy
			Proxy.new(Falcon::BadRequest, self.client_endpoints)
		end
		
		def redirection
			Redirection.new(Falcon::BadRequest, self.client_endpoints)
		end
		
		def call(controller)
			self.each do |name, host|
				if container = host.start
					controller << container
				end
			end

			proxy = hosts.proxy
			debug_trap = Async::IO::Trap.new(:USR1)

			profile = RubyProf::Profile.new(merge_fibers: true)

			controller << Async::Container::Forked.new do |task|
				Process.setproctitle("Falcon Proxy")
				
				server = Falcon::Server.new(
					proxy,
					Async::HTTP::URLEndpoint.parse(
						'https://0.0.0.0',
						reuse_address: true,
						ssl_context: hosts.ssl_context
					)
				)
				
				Async::Reactor.run do |task|
					task.async do
						debug_trap.install!
						$stderr.puts "Send `kill -USR1 #{Process.pid}` for detailed status :)"
						
						debug_trap.trap do
							task.reactor.print_hierarchy($stderr)
							# Async.logger.level = Logger::DEBUG
						end
					end
					
					task.async do |task|
						start_time = Async::Clock.now
						
						while true
							task.sleep(600)
							duration = Async::Clock.now - start_time
							puts "Handled #{proxy.count} requests; #{(proxy.count.to_f / duration.to_f).round(1)} requests per second."
						end
					end
					
					$stderr.puts "Starting server"
					server.run
				end
			end
		end
	end
end

require 'async/http/client'
require 'http/protocol/headers'

module Falcon
	module BadRequest
		def self.call(request)
			return Async::HTTP::Response[400, {}, []]
		end
		
		def self.close
		end
	end
	
	class Proxy < Async::HTTP::Middleware
		FORWARDED = 'forwarded'.freeze
		X_FORWARDED_FOR = 'x-forwarded-for'.freeze
		X_FORWARDED_PROTO = 'x-forwarded-proto'.freeze
		
		VIA = 'via'.freeze
		CONNECTION = ::HTTP::Protocol::CONNECTION
		
		HOP_HEADERS = [
			'connection',
			'keep-alive',
			'public',
			'proxy-authenticate',
			'transfer-encoding',
			'upgrade',
		]
		
		def initialize(app, hosts)
			super(app)
			
			@server_context = nil
			
			@hosts = hosts
			@clients = {}
			
			@count = 0
		end
		
		attr :count
		
		def close
			@clients.each_value(&:close)
			
			super
		end
		
		def connect(endpoint)
			@clients[endpoint] ||= Async::HTTP::Client.new(endpoint)
		end
		
		def lookup(request)
			# Trailing dot and port is ignored/normalized.
			if authority = request.authority.sub(/(\.)?(:\d+)?$/, '')
				return @hosts[authority]
			end
		end
		
		def prepare_headers(headers)
			if connection = headers[CONNECTION]
				headers.slice!(connection)
			end
			
			headers.slice!(HOP_HEADERS)
		end
		
		def prepare_request(request)
			forwarded = []
			
			if address = request.remote_address
				request.headers.add(X_FORWARDED_FOR, address.ip_address)
				forwarded << "for=#{address.ip_address}"
			end
			
			if scheme = request.scheme
				request.headers.add(X_FORWARDED_PROTO, scheme)
				forwarded << "proto=#{scheme}"
			end
			
			unless forwarded.empty?
				request.headers.add(FORWARDED, forwarded.join(';'))
			end
			
			request.headers.add(VIA, "#{request.version} #{self.class}")
			
			self.prepare_headers(request.headers)
			
			return request
		end
		
		def call(request)
			if endpoint = lookup(request)
				@count += 1
				
				request = self.prepare_request(request)
				
				client = connect(endpoint)
				
				client.call(request)
			else
				super
			end
		rescue
			Async.logger.error(self) {$!}
			return Async::HTTP::Response[502, {'content-type' => 'text/plain'}, ["#{$!.inspect}: #{$!.backtrace.join("\n")}"]]
		end
	end
end


require 'async/http/client'
require 'http/protocol/headers'

module Falcon
	module NotFound
		def self.call(request)
			return Async::HTTP::Response[404, {}, []]
		end
		
		def self.close
		end
	end
	
	class Redirection < Async::HTTP::Middleware
		def initialize(app, hosts)
			super(app)
			
			@hosts = hosts
		end
		
		def lookup(request)
			# Trailing dot and port is ignored/normalized.
			if authority = request.authority.sub(/(\.)?(:\d+)?$/, '')
				return @hosts[authority]
			end
		end
		
		def call(request)
			if endpoint = lookup(request)
				location = "https://#{request.authority}#{request.path}"
				
				return Async::HTTP::Response[301, {'location' => location}, []]
			else
				super
			end
		end
	end
end

require 'async/logger'
require 'async/http/statistics'

module Falcon
	class Verbose < Async::HTTP::Middleware
		def initialize(app, logger = Async.logger)
			super(app)
			
			@logger = logger
		end
		
		def annotate(request)
			task = Async::Task.current
			address = request.remote_address
			
			@logger.debug(request.authority) {"#{request.method} #{request.path} #{request.version} from #{address.inspect}"}
			
			# if ENV['REQUEST_HEADERS']
			# 	@logger.debug(request.authority) {request.headers.inspect}
			# end
			
			task.annotate("#{request.method} #{request.path} from #{address.inspect}")
		end
		
		def call(request)
			annotate(request)
			
			statistics = Async::HTTP::Statistics.start
			
			response = super
			
			statistics.wrap(response) do |statistics, error|
				@logger.info(request.authority) {"#{request.method} #{request.path} #{request.version} -> #{response.status}; #{statistics.inspect}"}
				
				# if ENV['RESPONSE_HEADERS']
				# 	@logger.info response.headers.inspect
				# end
				
				@logger.error(request.authority) {"#{error.class}: #{error.message}"} if error
			end
			
			return response
		end
	end
end

require 'async/io/buffer'

require 'async/http/body'
require 'async/http/body/rewindable'

module Falcon
	module Adapters
		# The input stream is an IO-like object which contains the raw HTTP POST data. When applicable, its external encoding must be “ASCII-8BIT” and it must be opened in binary mode, for Ruby 1.9 compatibility. The input stream must respond to gets, each, read and rewind.
		class Input
			def initialize(body)
				@body = body
				
				# Will hold remaining data in `#read`.
				@buffer = nil
				@finished = @body.nil?
			end
			
			attr :body
			
			# each must be called without arguments and only yield Strings.
			def each(&block)
				return to_enum unless block_given?
				
				while chunk = gets
					yield chunk
				end
			end
			
			# rewind must be called without arguments. It rewinds the input stream back to the beginning. It must not raise Errno::ESPIPE: that is, it may not be a pipe or a socket. Therefore, handler developers must buffer the input data into some rewindable object if the underlying input stream is not rewindable.
			# @return [Boolean] whether the body could be rewound.
			def rewind
				if @body and @body.respond_to? :rewind
					# If the body is not rewindable, this will fail.
					@body.rewind
					@buffer = nil
					@finished = false
					
					return true
				end
				
				return false
			end
			
			# read behaves like IO#read. Its signature is read([length, [buffer]]). If given, length must be a non-negative Integer (>= 0) or nil, and buffer must be a String and may not be nil. If length is given and not nil, then this method reads at most length bytes from the input stream. If length is not given or nil, then this method reads all data until EOF. When EOF is reached, this method returns nil if length is given and not nil, or “” if length is not given or is nil. If buffer is given, then the read data will be placed into buffer instead of a newly created String object.
			# @param length [Integer] the amount of data to read
			# @param buffer [String] the buffer which will receive the data
			# @return a buffer containing the data
			def read(length = nil, buffer = nil)
				buffer ||= Async::IO::Buffer.new
				buffer.clear
				
				until buffer.bytesize == length
					@buffer = read_next if @buffer.nil?
					break if @buffer.nil?
					
					remaining_length = length - buffer.bytesize if length
					
					if remaining_length && remaining_length < @buffer.bytesize
						# We know that we are not going to reuse the original buffer.
						# But byteslice will generate a hidden copy. So let's freeze it first:
						@buffer.freeze
						
						buffer << @buffer.byteslice(0, remaining_length)
						@buffer = @buffer.byteslice(remaining_length, @buffer.bytesize)
					else
						buffer << @buffer
						@buffer = nil
					end
				end
				
				return nil if buffer.empty? && length && length > 0
				
				return buffer
			end
			
			def eof?
				@finished and @buffer.nil?
			end
			
			# gets must be called without arguments and return a string, or nil on EOF.
			# @return [String, nil] The next chunk from the body.
			def gets
				if @buffer.nil?
					return read_next
				else
					buffer = @buffer
					@buffer = nil
					return buffer
				end
			end
			
			# close must never be called on the input stream. huh?
			def close
				@body&.close
			end
			
			private
			
			def read_next
				return nil if @finished
				
				if chunk = @body.read
					return chunk
				else
					@finished = true
					return nil
				end
			end
		end
	end
end


# Copyright, 2018, by Samuel G. D. Williams. <http://www.codeotaku.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

require 'async/http/body/readable'
require 'async/http/body/file'

module Falcon
	module Adapters
		# Wraps the rack response body.
		# The Body must respond to each and must only yield String values. The Body itself should not be an instance of String, as this will break in Ruby 1.9. If the Body responds to close, it will be called after iteration. If the body is replaced by a middleware after action, the original body must be closed first, if it responds to close. If the Body responds to to_path, it must return a String identifying the location of a file whose contents are identical to that produced by calling each; this may be used by the server as an alternative, possibly more efficient way to transport the response. The Body commonly is an Array of Strings, the application instance itself, or a File-like object.
		class Output < Async::HTTP::Body::Readable
			CONTENT_LENGTH = 'content-length'.freeze
			
			# Wraps an array into a buffered body.
			def self.wrap(status, headers, body)
				# In no circumstance do we want this header propagating out:
				if content_length = headers.delete(CONTENT_LENGTH)
					# We don't really trust the user to provide the right length to the transport.
					content_length = Integer(content_length)
				end
				
				if body.is_a?(Async::HTTP::Body::Readable)
					return body
				elsif status == 200 and body.respond_to?(:to_path)
					# Don't mangle partial responsese (206)
					return Async::HTTP::Body::File.open(body.to_path)
				else
					return self.new(headers, body, content_length)
				end
			end
			
			def initialize(headers, body, length)
				@length = length
				@body = body
				
				# An enumerator over the rack response body:
				@chunks = body.to_enum(:each)
			end
			
			# The rack response body.
			attr :body
			
			# The content length of the rack response body.
			attr :length
			
			def empty?
				@length == 0 or (@body.respond_to?(:empty?) and @body.empty?)
			end
			
			def close(error = nil)
				if @body and @body.respond_to?(:close)
					@body.close
					@body = nil
				end
				
				@chunks = nil
				
				super
			end
			
			def read
				if @chunks
					return @chunks.next
				end
			rescue StopIteration
				nil
			end
			
			def inspect
				"\#<#{self.class} length=#{@length.inspect} body=#{@body.class}>"
			end
		end
	end
end

require 'rack'

require_relative 'input'
require_relative 'response'

require 'async/logger'

module Falcon
	module Adapters
		class Rack
			HTTP_X_FORWARDED_PROTO = 'HTTP_X_FORWARDED_PROTO'.freeze
			REMOTE_ADDR = 'REMOTE_ADDR'.freeze
			
			CONTENT_TYPE = 'CONTENT_TYPE'.freeze
			CONTENT_LENGTH = 'CONTENT_LENGTH'.freeze
			
			def initialize(app, logger = Async.logger)
				@app = app
				
				raise ArgumentError, "App must be callable!" unless @app.respond_to?(:call)
				
				@logger = logger
			end
			
			# Rack separates multiple headers with the same key, into a single field with multiple "lines".
			def unwrap_headers(headers, env)
				headers.each do |key, value|
					http_key = "HTTP_#{key.upcase.tr('-', '_')}"
					
					if current_value = env[http_key]
						env[http_key] = "#{current_value}\n#{value}"
					else
						env[http_key] = value
					end
				end
			end
			
			# Process the incoming request into a valid rack env.
			def unwrap_request(request, env)
				if content_type = request.headers.delete('content-type')
					env[CONTENT_TYPE] = content_type
				end
				
				# In some situations we don't know the content length, e.g. when using chunked encoding, or when decompressing the body.
				if body = request.body and length = body.length
					env[CONTENT_LENGTH] = length.to_s
				end
				
				self.unwrap_headers(request.headers, env)
				
				# HTTP/2 prefers `:authority` over `host`, so we do this for backwards compatibility.
				env[::Rack::HTTP_HOST] ||= request.authority
				
				# This is the HTTP/1 header for the scheme of the request and is used by Rack.
				# Technically it should use the Forwarded header but this is not common yet.
				# https://tools.ietf.org/html/rfc7239#section-5.4
				# https://github.com/rack/rack/issues/1310
				env[HTTP_X_FORWARDED_PROTO] ||= request.scheme
				
				if remote_address = request.remote_address
					env[REMOTE_ADDR] = remote_address.ip_address if remote_address.ip?
				end
			end
			
			def make_response(request, status, headers, body)
				@logger.debug(request) {"Rack response: #{status} #{headers.inspect} #{body.class}"}
				
				return Response.wrap(status, headers, body)
			end
			
			def call(request)
				request_path, query_string = request.path.split('?', 2)
				server_name, server_port = (request.authority || '').split(':', 2)
				
				env = {
					::Rack::RACK_VERSION => [2, 0, 0],
					
					::Rack::RACK_INPUT => Input.new(request.body),
					::Rack::RACK_ERRORS => $stderr,
					
					::Rack::RACK_MULTITHREAD => true,
					::Rack::RACK_MULTIPROCESS => true,
					::Rack::RACK_RUNONCE => false,
					
					# The HTTP request method, such as “GET” or “POST”. This cannot ever be an empty string, and so is always required.
					::Rack::REQUEST_METHOD => request.method,
					
					# The initial portion of the request URL's “path” that corresponds to the application object, so that the application knows its virtual “location”. This may be an empty string, if the application corresponds to the “root” of the server.
					::Rack::SCRIPT_NAME => '',
					
					# The remainder of the request URL's “path”, designating the virtual “location” of the request's target within the application. This may be an empty string, if the request URL targets the application root and does not have a trailing slash. This value may be percent-encoded when originating from a URL.
					::Rack::PATH_INFO => request_path,
					
					# The portion of the request URL that follows the ?, if any. May be empty, but is always required!
					::Rack::QUERY_STRING => query_string || '',
					
					# The server protocol (e.g. HTTP/1.1):
					::Rack::SERVER_PROTOCOL => request.version,
					
					# The request scheme:
					::Rack::RACK_URL_SCHEME => request.scheme,
					
					# I'm not sure what sane defaults should be here:
					::Rack::SERVER_NAME => server_name || '',
					::Rack::SERVER_PORT => server_port || '',
				}
				
				self.unwrap_request(request, env)
				
				if request.hijack?
					env[::Rack::RACK_IS_HIJACK] = true
					
					env[::Rack::RACK_HIJACK] = lambda do
						wrapper = request.hijack
						
						# We dup this as it might be taken out of the normal control flow, and the io will be closed shortly after returning from this method.
						io = wrapper.io.dup
						wrapper.close
						
						# This is implicitly returned:
						env[::Rack::RACK_HIJACK_IO] = io
					end
				else
					env[::Rack::RACK_IS_HIJACK] = false
				end
				
				status, headers, body = @app.call(env)
				
				# Partial hijack is not supported/tested.
				# if hijack = headers.delete('rack.hijack')
				# 	body = Async::HTTP::Body::Writable.new
				# 
				# 	Task.current.async do
				# 		hijack.call(body)
				# 	end
				# 	return nil
				# end
				
				# if env['rack.hijack_io']
				# 	return nil
				# end
				
				return make_response(request, status, headers, body)
			rescue => exception
				@logger.error "#{exception.class}: #{exception.message}\n\t#{$!.backtrace.join("\n\t")}"
				
				return failure_response(exception)
			end
			
			def failure_response(exception)
				Async::HTTP::Response.for_exception(exception)
			end
		end
	end
end

require 'async/http/body/rewindable'

module Falcon
	module Adapters
		# Content type driven input buffering.
		class Rewindable < Async::HTTP::Middleware
			BUFFERED_MEDIA_TYPES = %r{
				application/x-www-form-urlencoded|
				multipart/form-data|
				multipart/related|
				multipart/mixed
			}x
			
			POST = 'POST'.freeze
			
			def initialize(app)
				super(app)
			end
			
			def needs_rewind?(request)
				content_type = request.headers['content-type']
				
				if request.method == POST and content_type.nil?
					return true
				end
				
				if BUFFERED_MEDIA_TYPES =~ content_type
					return true
				end
				
				return false
			end
			
			# Wrap the request body in a rewindable buffer.
			# @return [Async::HTTP::Response] the response.
			def call(request)
				if body = request.body and needs_rewind?(request)
					request.body = Async::HTTP::Body::Rewindable.new(body)
				end
				
				return super
			end
		end
	end
end

require_relative '../server'
require_relative '../endpoint'

require 'async/container'
require 'async/io/trap'
require 'async/io/host_endpoint'
require 'async/io/shared_endpoint'
require 'async/io/ssl_endpoint'

require 'samovar'

require 'rack/builder'
require 'rack/server'

module Falcon
	module Command
		class Serve < Samovar::Command
			self.description = "Run an HTTP server."
			
			options do
				option '-b/--bind <address>', "Bind to the given hostname/address", default: "https://localhost:9292"
				
				option '-p/--port <number>', "Override the specified port", type: Integer
				option '-h/--hostname <hostname>', "Specify the hostname which would be used for certificates, etc."
				
				option '-c/--config <path>', "Rackup configuration file to load", default: 'config.ru'
				option '-n/--concurrency <count>', "Number of processes to start", default: Async::Container.hardware_concurrency, type: Integer
				
				option '--forked | --threaded', "Select a specific concurrency model", key: :container, default: :forked
			end
			
			def container_class
				case @options[:container]
				when :threaded
					require 'async/container/threaded'
					return Async::Container::Threaded
				when :forked
					require 'async/container/forked'
					return Async::Container::Forked
				end
			end
			
			def load_app(verbose)
				rack_app, options = Rack::Builder.parse_file(@options[:config])
				
				return Server.middleware(rack_app, verbose: verbose), options
			end
			
			def endpoint_options
				# Oh, for Hash#slice(keys...)
				options = {}
				
				if @options[:hostname]
					options[:hostname] = @options[:hostname]
				end
				
				if @options[:port]
					options[:port] = @options[:port]
				end
				
				return options
			end
			
			def client_endpoint
				Async::HTTP::URLEndpoint.parse(@options[:bind], **endpoint_options)
			end
			
			def client
				Async::HTTP::Client.new(client_endpoint)
			end
			
			def run(verbose = false)
				app, _ = load_app(verbose)
				
				endpoint = Endpoint.parse(@options[:bind], **endpoint_options)
				
				bound_endpoint = Async::Reactor.run do
					Async::IO::SharedEndpoint.bound(endpoint)
				end.result
				
				Async.logger.info "Falcon taking flight! Binding to #{endpoint} [#{container_class} with concurrency: #{@options[:concurrency]}]"
				
				debug_trap = Async::IO::Trap.new(:USR1)
				
				container_class.new(concurrency: @options[:concurrency], name: "Falcon Server") do |task, instance|
					task.async do
						debug_trap.install!
						Async.logger.info "Send `kill -USR1 #{Process.pid}` for detailed status :)"
						
						debug_trap.trap do
							task.reactor.print_hierarchy($stderr)
						end
					end
					
					server = Falcon::Server.new(app, bound_endpoint, endpoint.protocol, endpoint.scheme)
					
					server.run
					
					task.children.each(&:wait)
				end
			end
			
			def invoke(parent)
				container = run(parent.verbose?)
				
				container.wait
			end
		end
	end
end

require_relative '../server'
require_relative '../endpoint'
require_relative '../hosts'

require 'async/container'
require 'async/container/controller'

require 'async/io/host_endpoint'
require 'async/io/shared_endpoint'
require 'async/io/ssl_endpoint'

require 'samovar'

require 'rack/builder'
require 'rack/server'

module Falcon
	module Command
		class Virtual < Samovar::Command
			self.description = "Run an HTTP server with one or more virtual hosts."
			
			options do
				option '--bind-insecure <address>', "Bind redirection to the given hostname/address", default: "http://localhost"
				option '--bind-secure <address>', "Bind proxy to the given hostname/address", default: "https://localhost"
				
				option '--self-signed', "Use self-signed SSL", default: false
			end
			
			many :sites
			
			CONFIG_RU = "config.ru"
			
			def load_app(path, verbose)
				config = File.join(path, CONFIG_RU)
				
				rack_app, options = Rack::Builder.parse_file(config)
				
				return Server.middleware(rack_app, verbose: verbose), options
			end
			
			def client
				Async::HTTP::Client.new(client_endpoint)
			end
			
			def run(verbose = false)
				hosts = Falcon::Hosts.new
				root = Dir.pwd
				
				sites.each do |path|
					name = File.basename(path)
					
					hosts.add(name) do |host|
						host.app_root = File.expand_path(path, root)
						
						if @options[:self_signed]
							host.self_signed!(name)
						else
							host.ssl_certificate_path = File.join(path, "ssl", "fullchain.pem")
							host.ssl_key_path = File.join(path, "ssl", "privkey.pem")
						end
					end
				end
				
				controller = Async::Container::Controller.new
				
				hosts.each do |name, host|
					if container = host.start
						controller << container
					end
				end
				
				controller << Async::Container::Forked.new do |task|
					proxy = hosts.proxy
					secure_endpoint = Async::HTTP::URLEndpoint.parse(@options[:bind_secure], ssl_context: hosts.ssl_context)
					
					Process.setproctitle("Falcon Proxy")
					
					proxy_server = Falcon::Server.new(proxy, secure_endpoint)
					
					proxy_server.run
				end
				
				controller << Async::Container::Forked.new do |task|
					redirection = hosts.redirection
					insecure_endpoint = Async::HTTP::URLEndpoint.parse(@options[:bind_insecure])
					
					Process.setproctitle("Falcon Redirector")
					
					redirection_server = Falcon::Server.new(redirection, insecure_endpoint)
					
					redirection_server.run
				end
				
				Process.setproctitle("Falcon Controller")
				
				return controller
			end
			
			def invoke(parent)
				container = run(parent.verbose?)
				
				container.wait
			end
		end
	end
end

require 'rack/handler'

require_relative '../../falcon'

require 'async/io/host_endpoint'

module Rack
	module Handler
		module Falcon
			SCHEME = "http".freeze
			
			def self.endpoint_for(**options)
				host = options[:Host] || 'localhost'
				port = Integer(options[:Port] || 9292)
				
				return Async::IO::Endpoint.tcp(host, port)
			end
			
			def self.run(app, **options)
				endpoint = endpoint_for(**options)
				
				app = ::Falcon::Adapters::Rack.new(app)
				app = ::Falcon::Adapters::Rewindable.new(app)
				
				server = ::Falcon::Server.new(app, endpoint, Async::HTTP::Protocol::HTTP1, SCHEME)
				
				Async::Reactor.run do
					server.run
				end
			end
		end
		
		register :falcon, Falcon
	end
end

if ENV['COVERAGE'] || ENV['TRAVIS']
	begin
		require 'simplecov'
		
		SimpleCov.start do
			add_filter "/spec/"
		end
		
		if ENV['TRAVIS']
			require 'coveralls'
			Coveralls.wear!
		end
	rescue LoadError
		warn "Could not load simplecov: #{$!}"
	end
end

require "bundler/setup"
require "falcon"

require "async/rspec"
require "async/http/url_endpoint"

RSpec.shared_context Falcon::Server do
	include_context Async::RSpec::Reactor
	
	let(:protocol) {Async::HTTP::Protocol::HTTP1}
	let(:endpoint) {Async::HTTP::URLEndpoint.parse('http://127.0.0.1:9294', reuse_port: true)}
	let!(:client) {Async::HTTP::Client.new(endpoint, protocol)}
	
	let!(:server_task) do
		server_task = reactor.async do
			server.run
		end
	end
	
	after(:each) do
		server_task.stop
		client.close
	end
	
	let(:app) do
		lambda do |env|
			[200, {}, []]
		end
	end
	
	let(:server) do
		Falcon::Server.new(
			Falcon::Adapters::Rewindable.new(
				Falcon::Adapters::Rack.new(app)
			),
			endpoint, protocol
		)
	end
end

RSpec.configure do |config|
	# Enable flags like --only-failures and --next-failure
	config.example_status_persistence_file_path = ".rspec_status"

	config.expect_with :rspec do |c|
		c.syntax = :expect
	end
end

require 'falcon/server'
require 'async/http/client'

require 'async/process'

RSpec.describe Falcon::Server do
	include_context Async::RSpec::Reactor
	
	let(:config_path) {File.join(__dir__, "config.ru")}
	
	let(:server) {'falcon'} # of course :)
	
	let(:endpoint) {Async::HTTP::URLEndpoint.parse("http://localhost:9290")}
	let(:client) {Async::HTTP::Client.new(endpoint)}
	
	it "can start server" do
		server_task = reactor.async do
			Async::Process.spawn("rackup", "--server", server, "--host", endpoint.hostname, "--port", endpoint.port.to_s, config_path)
		end
		
		Async::Task.current.sleep 2
		
		response = client.post("/", {}, ["Hello World"])
	
		expect(response).to be_success
		expect(response.read).to be == "Hello World"
		
		client.close
		server_task.stop
	end
end


require 'falcon/adapters/input'

RSpec.describe Falcon::Adapters::Input do
	include_context Async::RSpec::Memory

	context 'with body' do
		let(:sample_data) {%w{The quick brown fox jumped over the lazy dog}}
		let(:body) {Async::HTTP::Body::Buffered.new(sample_data)}
		
		subject {described_class.new(body)}
		
		context '#read(length, buffer)' do
			let(:buffer) {Async::IO::Buffer.new}
			let(:expected_output) {sample_data.join}
			
			it "can close input" do
				expect(body).to receive(:close).and_call_original
				subject.close
			end
			
			it "can read partial input" do
				expect(subject.read(3, buffer)).to be == "The"
				expect(buffer).to be == "The"
			end
			
			it "can read all input" do
				expect(subject.read(expected_output.bytesize, buffer)).to be == expected_output
				expect(buffer).to be == expected_output
				
				# Not sure about this. The next read will not produce any additional data, but we don't konw if we are at EOF yet.
				expect(subject).to_not be_eof
				
				expect(subject.read(expected_output.bytesize, buffer)).to be == nil
				expect(buffer).to be == ""
				
				expect(subject).to be_eof
			end
			
			context "with large body" do
				# Allocate 5 chunks, each containing 1 MB of data.
				let(:sample_data) {Array.new(5) {|i| "#{i}".b * 1024*1024}}
				
				it "allocates expected amount of memory" do
					subject
					
					expect do
						subject.read(10*1024, buffer)
					end.to limit_allocations.of(String, count: 5, size: 1024*1024*0.9..1024*1024*1.1)
				end
			end
		end
		
		context '#read' do
			it "can read all input" do
				expect(subject.read).to be == sample_data.join
				expect(subject.read).to be == ""
			end
			
			it "can read no input" do
				expect(subject.read(0)).to be == ""
			end
			
			it "can read partial input" do
				2.times do
					expect(subject.read(3)).to be == "The"
					expect(subject.read(3)).to be == "qui"
					expect(subject.read(3)).to be == "ckb"
					expect(subject.read(3)).to be == "row"
					
					subject.rewind
				end
				
				expect(subject.read(15)).to be == sample_data.join[0...15]
				expect(subject.read).to be == sample_data.join[15..-1]
				
				expect(subject.read(1)).to be == nil
				expect(subject).to be_eof
			end
			
			it "can read partial input with buffer" do
				buffer = String.new
				
				2.times do
					expect(subject.read(3, buffer)).to be == "The"
					expect(subject.read(3, buffer)).to be == "qui"
					expect(subject.read(3, buffer)).to be == "ckb"
					expect(subject.read(3, buffer)).to be == "row"
					
					expect(buffer).to be == "row"
					
					subject.rewind
				end
				
				data = subject.read(15, buffer)
				expect(data).to be == sample_data.join[0...15]
				expect(buffer).to equal(data)
				
				expect(subject.read).to be == sample_data.join[15..-1]
				
				expect(subject.read(1, buffer)).to be == nil
				expect(buffer).to be == ""
				
				expect(subject).to be_eof
			end
			
			context "with large body" do
				let(:sample_data) { Array.new(5) { |i| "#{i}".b * 1024*1024 } }
				
				it "allocates expected amount of memory" do
					subject
					
					expect {
						subject.read.clear
					}.to limit_allocations.of(String, count: 11, size: 5242885)
				end
			end
		end
		
		context '#gets' do
			it "can read chunks" do
				sample_data.each do |chunk|
					expect(subject.gets).to be == chunk
				end
				
				expect(subject.gets).to be == nil
			end
			
			it "returns remainder after calling #read" do
				expect(subject.read(4)).to be == "Theq"
				expect(subject.gets).to be == "uick"
				expect(subject.read(4)).to be == "brow"
				expect(subject.gets).to be == "n"
			end
		end
		
		context '#each' do
			it "can read chunks" do
				subject.each.with_index do |chunk, index|
					expect(chunk).to be == sample_data[index]
				end
			end
		end
		
		context '#eof?' do
			it "should not be at end of file" do
				expect(subject).to_not be_eof
			end
		end
		
		context '#rewind' do
			it "reads same chunk again" do
				expect(subject.gets).to be == "The"
				
				subject.rewind
				expect(subject.gets).to be == "The"
			end
			
			it "clears unread buffer" do
				expect(subject.gets).to be == "The"
				expect(subject.read(2)).to be == "qu"
				
				subject.rewind
				
				expect(subject.read(3)).to be == "The"
			end
		end
	end
	
	context 'without body' do
		subject {described_class.new(nil)}
		
		context '#read(length, buffer)' do
			let(:buffer) {Async::IO::Buffer.new}
			
			it "can read no input" do
				expect(subject.read(0, buffer)).to be == ""
				expect(buffer).to be == ""
			end
			
			it "can read partial input" do
				expect(subject.read(2, buffer)).to be == nil
				expect(buffer).to be == ""
			end
		end
		
		context '#read' do
			it "can read all input" do
				expect(subject.read).to be == ""
			end
			
			it "can read no input" do
				expect(subject.read(0)).to be == ""
			end
			
			it "can read partial input" do
				expect(subject.read(2)).to be_nil
			end
		end
		
		context '#gets' do
			it "can read chunks" do
				expect(subject.gets).to be_nil
			end
		end
		
		context '#eof?' do
			it "should be at end of file" do
				expect(subject).to be_eof
			end
		end
	end
end


require 'falcon/adapters/output'

RSpec.describe Falcon::Adapters::Output do
	context 'with empty body' do
		subject {described_class.new({}, [], nil)}
		
		it "should be empty?" do
			expect(subject).to be_empty
		end
	end
	
	context 'with single string body' do
		subject {described_class.new({}, ["Hello World"], nil)}
		
		it "should not be empty?" do
			expect(subject).to_not be_empty
		end
	end

end

require 'falcon/server'
require 'async/websocket/server'
require 'async/websocket/client'

RSpec.describe Falcon::Adapters::Rack do
	context '#unwrap_headers' do
		subject {described_class.new(lambda{})}
		
		let(:fields) {[['cookie', 'a=b'], ['cookie', 'x=y']]}
		let(:env) {Hash.new}
		
		it "should merge duplicate headers" do
			subject.unwrap_headers(fields, env)
			
			expect(env).to be == {'HTTP_COOKIE' => "a=b\nx=y"}
		end
	end
	
	context 'HTTP_HOST', timeout: 1 do
		include_context Falcon::Server
		let(:protocol) {Async::HTTP::Protocol::HTTP2}
		
		let(:app) do
			lambda do |env|
				[200, {}, ["HTTP_HOST: #{env['HTTP_HOST']}"]]
			end
		end
		
		let(:response) {client.get("/")}
		
		it "get valid HTTP_HOST" do
			expect(response.read).to be == "HTTP_HOST: 127.0.0.1:9294"
		end
	end
	
	context 'websockets', timeout: 1 do
		include_context Falcon::Server
		
		let(:endpoint) {Async::HTTP::URLEndpoint.parse('ws://127.0.0.1:9294', reuse_port: true)}
		
		let(:app) do
			lambda do |env|
				Async::WebSocket::Server.open(env) do |connection|
					while message = connection.next_message
						connection.send_message(message)
					end
				end
				
				[200, {}, []]
			end
		end
		
		let(:test_message) do
			{
				"user" => "test",
				"status" => "connected",
			}
		end
		
		it "can send and receive messages using websockets" do
			socket = endpoint.connect
			connection = Async::WebSocket::Client.new(socket, endpoint.url.to_s)
			
			connection.send_message(test_message)
			
			message = connection.next_message
			expect(message).to be == test_message
			
			connection.close
			socket.close
		end
	end
end


# Copyright, 2018, by Samuel G. D. Williams. <http://www.codeotaku.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

require 'falcon/server'
require 'async/websocket/server'
require 'async/websocket/client'

RSpec.describe Falcon::Adapters::Response do
	context 'with multiple set-cookie headers' do
		subject {described_class.wrap(200, {'Set-Cookie' => "a\nb"}, [])}
		
		let(:fields) {subject.headers.fields}
		
		it "should generate multiple headers" do
			expect(fields).to include(['set-cookie', 'a'])
			expect(fields).to include(['set-cookie', 'b'])
		end
	end
	
	context 'with #to_path' do
		let(:body) {double}
		
		it "should generate file body" do
			expect(body).to receive(:to_path).and_return("/dev/null")
			
			response = described_class.wrap(200, {}, body)
			
			expect(response.body).to be_kind_of Async::HTTP::Body::File
		end
		
		it "should not modify partial responses" do
			response = described_class.wrap(206, {}, body)
			
			expect(response.body).to be_kind_of Falcon::Adapters::Output
		end
	end
	
	context 'with content-length' do
		it "should remove header" do
			response = described_class.wrap(200, {'Content-Length' => '4'}, ["1234"])
			
			expect(response.headers).to_not include('content-length')
		end
	end
end

require 'falcon/command/serve'

RSpec.describe Falcon::Command::Serve do
	it "can listen on specified port" do
		command = described_class[
			# "--bind", "http://localhost",
			"--port", 8090,
			"--config", File.expand_path("config.ru", __dir__),
		]
		
		container = command.run
		
		Async::Reactor.run do
			client = command.client
			
			response = client.get("/")
			expect(response).to be_success
			
			client.close
		end
		
		container.stop
	end
end

require 'falcon/proxy'

require 'async/http/client'
require 'async/http/url_endpoint'

RSpec.describe Falcon::Proxy do
	include_context Async::RSpec::Reactor
	
	subject do
		described_class.new(Falcon::BadRequest, {
			'www.google.com' => Async::HTTP::URLEndpoint.parse('https://www.google.com'),
			'www.yahoo.com' => Async::HTTP::URLEndpoint.parse('https://www.yahoo.com')
		})
	end
	
	let(:headers) {Async::HTTP::Headers['accept' => '*/*']}
	
	it 'can select client based on authority' do
		request = Async::HTTP::Request.new('https', 'www.google.com', 'GET', '/', nil, headers, nil)
		
		expect(request).to receive(:remote_address).and_return(Addrinfo.ip("127.0.0.1"))
		
		response = subject.call(request)
		response.finish
		
		expect(response).to_not be_failure
		
		expect(request.headers['x-forwarded-for']).to be == ["127.0.0.1"]
		
		subject.close
	end
	
	it 'defers if no host is available' do
		request = Async::HTTP::Request.new('www.groogle.com', 'GET', '/', nil, headers, nil)
		
		response = subject.call(request)
		response.finish
		
		expect(response).to be_failure
		
		subject.close
	end
end

require 'falcon/server'
require 'async/http/client'
require 'async/rspec/reactor'

require 'rack'

RSpec.describe Falcon::Server, timeout: 1 do
	include_context Falcon::Server
	
	context "http client" do
		let(:app) do
			lambda do |env|
				request = Rack::Request.new(env)
				
				if request.post?
					[200, {}, ["POST: #{request.POST.inspect}"]]
				else
					[200, {}, ["Hello World"]]
				end
			end
		end
		
		context "GET /" do
			let(:response) {client.get("/")}
			
			it "generates successful response" do
				expect(response).to be_success
				expect(response.read).to be == "Hello World"
			end
			
			it "generates server and date headers" do
				expect(response.headers).to include('date', 'server')
				
				expect(response.headers['server'].join).to include("falcon")
			end
		end
		
		it "can POST application/x-www-form-urlencoded" do
			response = client.post("/", {'content-type' => 'application/x-www-form-urlencoded'}, ['hello=world'])
			
			expect(response).to be_success
			expect(response.read).to be == 'POST: {"hello"=>"world"}'
		end
		
		it "can POST multipart/form-data" do
			response = client.post("/", {'content-type' => 'multipart/form-data; boundary=multipart'}, ["--multipart\r\n", "Content-Disposition: form-data; name=\"hello\"\r\n\r\n", "world\r\n", "--multipart--"])
			
			expect(response).to be_success
			expect(response.read).to be == 'POST: {"hello"=>"world"}'
		end
	end
	
	context ::Rack::BodyProxy do
		let(:callback) {Proc.new{}}
		let(:content) {Array.new}
		
		let(:app) do
			lambda do |env|
				body = ::Rack::BodyProxy.new(content, &callback)
				
				[200, {}, body]
			end
		end
		
		it "should close non-empty body" do
			content << "Hello World"
			
			expect(callback).to receive(:call).and_call_original
			
			expect(client.get("/", {}).read).to be == "Hello World"
		end
		
		it "should close empty body" do
			expect(callback).to receive(:call)
			
			expect(client.get("/", {}).read).to be nil
		end
	end
	
	context "broken middleware" do
		let(:app) do
			lambda do |env|
				raise RuntimeError, "Middleware is broken"
			end
		end
		
		it "results in a 500 error if middleware raises an exception" do
			response = client.get("/", {})
			
			expect(response).to_not be_success
			expect(response.status).to be == 500
			expect(response.read).to be =~ /RuntimeError: Middleware is broken/
		end
	end
end

require 'falcon/server'
require 'async/http/client'
require 'async/rspec/reactor'
require 'async/rspec/ssl'

require 'async/io/ssl_socket'

RSpec.describe "Falcon::Server with SSL", timeout: 1 do
	include_context Async::RSpec::Reactor
	
	include_context Async::RSpec::SSL::ValidCertificate
	include_context Async::RSpec::SSL::VerifiedContexts
	
	let(:protocol) {Async::HTTP::Protocol::HTTPS}
	
	let(:server_endpoint) {Async::HTTP::URLEndpoint.parse("https://localhost:6365", ssl_context: server_context)}
	let(:client_endpoint) {Async::HTTP::URLEndpoint.parse("https://localhost:6365", ssl_context: client_context)}
	
	let(:server) {Falcon::Server.new(Falcon::Adapters::Rack.new(app), server_endpoint, protocol)}
	let(:client) {Async::HTTP::Client.new(client_endpoint, protocol)}
	after(:each) {client.close}
	
	around(:each) do |example|
		server_task = reactor.async do
			server.run
		end
		
		begin
			example.run
		ensure
			server_task.stop
		end
	end
	
	context "basic middleware" do
		let(:app) do
			lambda do |env|
				[200, {}, ["Hello World"]]
			end
		end
		
		it "client can get resource" do
			response = client.get("/", {})
			
			expect(response).to be_success
			expect(response.read).to be == "Hello World"
		end
	end
end