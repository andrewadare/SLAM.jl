( function() {
  'use strict';

  /**
   * Simple assertion checker with error message. Usage example:
   * var a = 1;
   * var b = 2;
   * assert && assert(a > b, 'Predicate a>b violated. a,b = ' + a + ',' + b)
   *
   * @param  {bool} predicate - condition that can be evaluated to true/false
   * @param  {string} message   - error message
   */
  window.assert = function( predicate, message ) {
    if ( !predicate ) {
      console && console.log && console.log( 'Assertion failed: ' + message );
      throw new Error( 'Assertion failed: ' + message );
    }
  };

} )();