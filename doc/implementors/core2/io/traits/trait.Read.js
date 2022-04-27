(function() {var implementors = {};
implementors["io"] = [{"text":"impl&lt;IO&gt; Read for <a class=\"struct\" href=\"io/struct.ReaderWriter.html\" title=\"struct io::ReaderWriter\">ReaderWriter</a>&lt;IO&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;IO: <a class=\"trait\" href=\"io/trait.ByteReader.html\" title=\"trait io::ByteReader\">ByteReader</a>,&nbsp;</span>","synthetic":false,"types":["io::ReaderWriter"]},{"text":"impl&lt;IO&gt; Read for <a class=\"struct\" href=\"io/struct.Reader.html\" title=\"struct io::Reader\">Reader</a>&lt;IO&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;IO: <a class=\"trait\" href=\"io/trait.ByteReader.html\" title=\"trait io::ByteReader\">ByteReader</a>,&nbsp;</span>","synthetic":false,"types":["io::Reader"]},{"text":"impl&lt;'io, IO, L, B&gt; Read for <a class=\"struct\" href=\"io/struct.LockableIo.html\" title=\"struct io::LockableIo\">LockableIo</a>&lt;'io, IO, L, B&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;IO: Read + 'io + ?<a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a>,<br>&nbsp;&nbsp;&nbsp;&nbsp;L: for&lt;'a&gt; <a class=\"trait\" href=\"lockable/trait.Lockable.html\" title=\"trait lockable::Lockable\">Lockable</a>&lt;'a, IO&gt; + ?<a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/marker/trait.Sized.html\" title=\"trait core::marker::Sized\">Sized</a>,<br>&nbsp;&nbsp;&nbsp;&nbsp;B: <a class=\"trait\" href=\"https://doc.rust-lang.org/nightly/core/borrow/trait.Borrow.html\" title=\"trait core::borrow::Borrow\">Borrow</a>&lt;L&gt;,&nbsp;</span>","synthetic":false,"types":["io::LockableIo"]}];
implementors["serial_port"] = [{"text":"impl Read for <a class=\"struct\" href=\"serial_port/struct.SerialPort.html\" title=\"struct serial_port::SerialPort\">SerialPort</a>","synthetic":false,"types":["serial_port::SerialPort"]}];
implementors["stdio"] = [{"text":"impl&lt;'a&gt; Read for <a class=\"struct\" href=\"stdio/struct.StdioReadGuard.html\" title=\"struct stdio::StdioReadGuard\">StdioReadGuard</a>&lt;'a&gt;","synthetic":false,"types":["stdio::StdioReadGuard"]}];
if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()