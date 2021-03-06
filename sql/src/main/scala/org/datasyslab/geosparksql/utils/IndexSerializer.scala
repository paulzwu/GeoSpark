/**
  * FILE: IndexSerializer
  * PATH: org.datasyslab.geosparksql.utils.IndexSerializer
  * Copyright (c) GeoSpark Development Team
  *
  * MIT License
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  * SOFTWARE.
  */
package org.datasyslab.geosparksql.utils

import java.io.{ByteArrayInputStream, ByteArrayOutputStream}

import com.esotericsoftware.kryo.Kryo
import com.esotericsoftware.kryo.io.{Input, Output}
import com.vividsolutions.jts.index.SpatialIndex
import org.apache.spark.sql.catalyst.util.ArrayData

// This is a wrapper of GeoSpark core kryo serializer
object IndexSerializer {
  def serialize(index: SpatialIndex): Array[Byte] = {
    val out = new ByteArrayOutputStream()
    val kryo = new Kryo()
    val output = new Output(out)
    kryo.writeObject(output,index)
    output.close()
    return out.toByteArray
  }

  def deserialize(values: ArrayData): SpatialIndex = {
    val in = new ByteArrayInputStream(values.toByteArray())
    val kryo = new Kryo()
    val input = new Input(in)
    val spatialIndex = kryo.readObject(input, classOf[SpatialIndex])
    input.close()
    return spatialIndex.asInstanceOf[SpatialIndex]
  }

}
