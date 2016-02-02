$( function() {

  // Initiate a WebSocket connection
  var ws = new WebSocket( 'ws://' + window.location.host );

  // Set up the scene SVG element ASAP to avoid reflowing
  var width = 500;
  var height = 500;
  var xscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ 0, width ]);
  var yscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ height, 0 ]);
  var scene = d3.select( '.scene' )
      .attr( 'width', width )
      .attr( 'height', height );

  function drawWaypoints( data ) {
    var waypoints = scene.selectAll( 'circle' )
        .data( data )
      .enter().append( 'circle' )
        .attr( 'r', function() { return 0; }) // px
      .transition()
        .duration( 750 )
        .attr( 'r', function() { return 5; }) // px
        .attr( 'cx', function( d ) { return xscale( d.x ); })
        .attr( 'cy', function( d ) { return height - yscale( d.y ); }); 
  }

  // Handler for messages received from server
  // https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API/Writing_WebSocket_client_applications
  ws.onmessage = function( event ) {
    var msg = JSON.parse( event.data );
    switch( msg.type ) {
      case "waypoints":
        drawWaypoints( msg.data );
        break;
      case "landmarks":
        break;
    }
  }

  $( '#start' ).click( function() {
    var msg = {
      type: 'request',
      text: 'get_waypoints',
      id:   1,
      date: Date.now()
    };
  
    // Send the msg object as a JSON-formatted string.
    ws.send(JSON.stringify(msg));
    // ws.send( 'getnumber' );
  });


});


  // function changeUsername( username ) {
  //   you = username;
  //   connection.send( 'setusername:' + username );
  //   $( '#sayer span' ).html( username );
  // }

  // function sendMessage( message ) {
  //   connection.send( 'say:' + message );
  //   $( '#content' ).prepend( $( '<p class="sent"></p>' ).html( you + ': ' + message ) );
  // }

  // $( '#sayer input[type=submit]' ).click( function() {
  //   if ( $( '#sayer input[name=say]' ).val().replace( /\s/gi, '' ).length )
  //     sendMessage( $( '#sayer input[name=say]' ).val() );

  //   $( '#sayer input[name=say]' ).val( '' ).focus();
  // } );

  // $( '#sayer input[name=say]' ).keypress( function( e ) {
  //   if ( e.which === 13 )
  //     $( '#sayer input[type=submit]' ).click();
  // } );

  // $( '#pick_username' ).submit( function( e ) {
  //   var uname = $( this ).find( 'input.username' ).val();
  //   if ( !uname.replace( /\s/gi, '' ).length )
  //     alert( 'Please select a valid username' );
  //   else {
  //     changeUsername( uname );
  //     $( '#welcome' ).hide();
  //     $( '#chat' ).show();
  //   }
  //   e.stopImmediatePropagation();
  //   e.preventDefault();
  //   return false
  // } );

