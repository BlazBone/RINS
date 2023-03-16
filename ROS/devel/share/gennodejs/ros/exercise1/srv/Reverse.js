// Auto-generated. Do not edit!

// (in-package exercise1.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ReverseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.content = null;
    }
    else {
      if (initObj.hasOwnProperty('content')) {
        this.content = initObj.content
      }
      else {
        this.content = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReverseRequest
    // Serialize message field [content]
    bufferOffset = _serializer.string(obj.content, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReverseRequest
    let len;
    let data = new ReverseRequest(null);
    // Deserialize message field [content]
    data.content = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.content);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'exercise1/ReverseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2e84951ee6d0addf437bfddd5b19734';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #request constants
    #int8 FOO=1
    #int8 BAR=2
    #request fields
    string content
    #int32 y
    #another_pkg/AnotherMessage msg
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReverseRequest(null);
    if (msg.content !== undefined) {
      resolved.content = msg.content;
    }
    else {
      resolved.content = ''
    }

    return resolved;
    }
};

class ReverseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.comment = null;
    }
    else {
      if (initObj.hasOwnProperty('comment')) {
        this.comment = initObj.comment
      }
      else {
        this.comment = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReverseResponse
    // Serialize message field [comment]
    bufferOffset = _serializer.string(obj.comment, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReverseResponse
    let len;
    let data = new ReverseResponse(null);
    // Deserialize message field [comment]
    data.comment = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.comment);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'exercise1/ReverseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83609817d68993e8cc8571226bf4197d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #response constants
    #uint32 SECRET=123456
    #response fields
    #another_pkg/YetAnotherMessage val
    #CustomMessageDefinedInThisPackage value
    #int32 result
    string comment
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReverseResponse(null);
    if (msg.comment !== undefined) {
      resolved.comment = msg.comment;
    }
    else {
      resolved.comment = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ReverseRequest,
  Response: ReverseResponse,
  md5sum() { return 'a6ae0dfa99b6e1c22e0f4dd5d3d7311b'; },
  datatype() { return 'exercise1/Reverse'; }
};
